/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "camera_model.h"

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "def.h"
#include "log_utils.h"


CameraInfo cam_s;//用于segmentation线程的相机
CameraInfo cam_t;//用于tracking线程的相机
CameraInfo cam_v;//用于VIO线程的相机

bool is_stereo = true;
bool is_undistort_input = true;


vector<string> GetCameraPath(const string &config_path){
    auto pn = config_path.find_last_of('/');
    std::string config_dir = config_path.substr(0, pn);

    cv::FileStorage fs(config_path, cv::FileStorage::READ);

    if (!fs.isOpened()){
        return {};
    }

    vector<string> ans;

    if(!fs["cam0_calib"].isNone()){
        std::string cam0_calib;
        fs["cam0_calib"] >> cam0_calib;
        std::string cam0Path = config_dir + "/" + cam0_calib;
        ans.push_back(cam0Path);

        if(!fs["cam1_calib"].isNone()){
            std::string cam1_calib;
            fs["cam1_calib"] >> cam1_calib;
            std::string cam1Path = config_dir + "/" + cam1_calib;
            ans.push_back(cam1Path);
        }
    }

    return ans;
}

template<typename T>
string CvMatToStr(const cv::Mat &m){
    if(m.empty()){
        return {};
    }
    else if(m.channels()>1){
        return "CvMatToStr() input Mat has more than one channel";
    }
    else{
        string ans;
        for(int i=0;i<m.rows;++i){
            for(int j=0;j<m.cols;++j){
                ans += std::to_string(m.at<T>(i,j)) + " ";
            }
            ans+="\n";
        }
        return ans;
    }
}


void SetCameraIntrinsicByK(CameraInfo &cam){
    cam.fx0 = cam.K0.at<double>(0,0);
    cam.fy0 = cam.K0.at<double>(1,1);
    cam.cx0 = cam.K0.at<double>(0,2);
    cam.cy0 = cam.K0.at<double>(1,2);
    if(is_stereo){
        cam.fx1 = cam.K1.at<double>(0,0);
        cam.fy1 = cam.K1.at<double>(1,1);
        cam.cx1 = cam.K1.at<double>(0,2);
        cam.cy1 = cam.K1.at<double>(1,2);
    }
}


void InitOneCamera(const std::string& config_path,CameraInfo &cam){
    vector<string> cam_paths = GetCameraPath(config_path);
    if(cam_paths.empty()){
        cerr<<"FeatureTracker() GetCameraPath() not found camera config:"<<config_path<<endl;
        std::terminate();
    }
    //cam.cam0 = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam_paths[0]);
    cam.cam0 = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
            "/home/chen/CLionProjects/PL-VINS_ws/src/PL-VINS/config/euroc/loop.yaml"
            );

    cam.K0 = cam.cam0->initUndistortRectifyMap(cam.left_undist_map1,cam.left_undist_map2);
    Debugv("InitOneCamera() initUndistortRectifyMap K0:\n{}", CvMatToStr<float>(cam.K0));

    if(is_stereo){
        if(cam_paths.size()==1){
            cerr<<"FeatureTracker() GetCameraPath() not found right camera config:"<<config_path<<endl;
            std::terminate();
        }
        cam.cam1 = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam_paths[1]);
    }

    ///获取内参
    if(cam.cam0->modelType()==camodocal::Camera::ModelType::PINHOLE){
        //0 k1();1 k2();2 p1();3 p2();4 fx();5 fy();6 cx();7 cy();
        vector<double> left_cam_para;
        cam.cam0->writeParameters(left_cam_para);
        cam.K0 = ( cv::Mat_<double> ( 3,3 ) << left_cam_para[4], 0.0, left_cam_para[6], 0.0, left_cam_para[5], left_cam_para[7], 0.0, 0.0, 1.0 );
        cam.D0 = ( cv::Mat_<double> ( 4,1 ) << left_cam_para[0], left_cam_para[1], left_cam_para[2], left_cam_para[3]);//畸变系数 k1 k2 p1 p2

        if(is_stereo){
            vector<double> right_cam_para;
            cam.cam1->writeParameters(right_cam_para);
            cam.K1 = ( cv::Mat_<double> ( 3,3 ) << right_cam_para[4], 0.0, right_cam_para[6], 0.0, right_cam_para[5], right_cam_para[7], 0.0, 0.0, 1.0 );
            cam.D1 = ( cv::Mat_<double> ( 4,1 ) << right_cam_para[0], right_cam_para[1], right_cam_para[2], right_cam_para[3]);//畸变系数 k1 k2 p1 p2
        }

        SetCameraIntrinsicByK(cam);
    }
    else if(cam.cam0->modelType()==camodocal::Camera::ModelType::MEI){
        cerr<<"InitOneCamera not implement"<<endl;
        std::terminate();
    }
    else{
        cerr<<"InitOneCamera not implement"<<endl;
        std::terminate();
    }


    ///设置是否对输入图像进行去畸变处理

    ///获取去畸变的映射矩阵
    if(is_undistort_input){

        cam.K0 = cam.cam0->initUndistortRectifyMap(cam.left_undist_map1,cam.left_undist_map2);
        cam.D0 = ( cv::Mat_<double> ( 4,1 ) << 0, 0, 0, 0);//畸变系数 k1 k2 p1 p2

        Debugv("InitOneCamera() initUndistortRectifyMap K0:\n{}", CvMatToStr<float>(cam.K0));

        //cv::Size image_size(left_cam_dl->imageWidth(),left_cam_dl->imageHeight());
        //cv::initUndistortRectifyMap(left_K,left_D,cv::Mat(),left_K,image_size,
        //                            CV_16SC2,left_undist_map1,left_undist_map2);
        //dynamic_cast<camodocal::CataCamera*>(left_cam_dl.get())->initUndistortMap(left_undist_map1,left_undist_map2);

        if(is_stereo){
            cam.K1 = cam.cam1->initUndistortRectifyMap(cam.right_undist_map1,cam.right_undist_map2);
            cam.D1 = ( cv::Mat_<double> ( 4,1 ) << 0, 0, 0, 0);//畸变系数 k1 k2 p1 p2
            Debugv("InitOneCamera() initUndistortRectifyMap K1:\n{}", CvMatToStr<float>(cam.K1));
            //cv::Size image_size_2(right_cam_dl->imageWidth(),right_cam_dl->imageHeight());
            //cv::initUndistortRectifyMap(right_K,right_D,cv::Mat(),right_K,image_size_2,
            //                           CV_16SC2,right_undist_map1,right_undist_map2);

            //dynamic_cast<camodocal::CataCamera*>(right_cam_dl.get())->initUndistortMap(right_undist_map1,right_undist_map2);
        }

        ///由于会对整张图像进行去畸变，因此重新设置相机的内参
        if(cam.cam0->modelType()==camodocal::Camera::ModelType::PINHOLE){
            SetCameraIntrinsicByK(cam);
            //0 k1();1 k2();2 p1();3 p2();4 fx();5 fy();6 cx();7 cy();
            vector<double> left_cam_para = {0,0,0,0,cam.fx0,cam.fy0,cam.cx0,cam.cy0};
            cam.cam0->readParameters(left_cam_para);
            if(is_stereo){
                vector<double> right_cam_para = {0,0,0,0,cam.fx1,cam.fy1,cam.cx1,cam.cy1};
                cam.cam1->readParameters(right_cam_para);
            }
        }
        else if(cam.cam0->modelType()==camodocal::Camera::ModelType::MEI){
            cerr<<"InitOneCamera not implement"<<endl;
            std::terminate();
        }
        else{
            cerr<<"InitOneCamera not implement"<<endl;
            std::terminate();
        }

    }

    Debugv("InitOneCamera() left cam intrinsic fx:{} fy:{} cx:{} cy:{}",cam.fx0,cam.fy0,cam.cx0,cam.cy0);
    if(is_stereo){
        Debugv("InitOneCamera() right cam intrinsic fx:{} fy:{} cx:{} cy:{}",cam.fx1,cam.fy1,cam.cx1,cam.cy1);
    }

}


void InitCamera(const std::string& config_path){
    InitOneCamera(config_path,cam_s);
    //InitOneCamera(config_path,cam_t);
    //InitOneCamera(config_path,cam_v);
    cam_t = cam_s;
    cam_v = cam_s;
}




