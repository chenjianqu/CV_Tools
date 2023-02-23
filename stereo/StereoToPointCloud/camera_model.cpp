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
#include <spdlog/spdlog.h>

#include "def.h"
#include "kitti_utils.h"





void InitCamera(const std::string& kitti_calib_path){

    ///直接从kitti的参数文件中读取

    auto calib_map = kitti::ReadCalibFile(kitti_calib_path);

    cam0 = std::make_shared<PinHoleCamera>();
    cam0->image_width = 1242;
    cam0->image_height = 375;
    cam0->fx = calib_map["P2"](0,0);
    cam0->fy = calib_map["P2"](1,1);
    cam0->cx = calib_map["P2"](0,2);
    cam0->cy = calib_map["P2"](1,2);
    cam0->baseline = 0;
    double baseline_2 = calib_map["P2"](0,3) / (- cam0->fx);
    cout<<"baseline_2:"<<baseline_2<<endl;

    cam1 = std::make_shared<PinHoleCamera>();
    cam1->image_width = 1242;
    cam1->image_height = 375;
    cam1->fx = calib_map["P3"](0,0);
    cam1->fy = calib_map["P3"](1,1);
    cam1->cx = calib_map["P3"](0,2);
    cam1->cy = calib_map["P3"](1,2);
    double baseline_3 = calib_map["P3"](0,3) / (- cam1->fx);
    cout<<"baseline_3:"<<baseline_3<<endl;

    cam1->baseline = baseline_3 - baseline_2;

    cout<<"P2:\n"<<calib_map["P2"]<<endl;
    cout<<"P3:\n"<<calib_map["P3"]<<endl;


    cam0->inv_k11 = 1.f / cam0->fx;
    cam0->inv_k22 = 1.f / cam0->fy;
    cam0->inv_k13 = -cam0->cx / cam0->fx;
    cam0->inv_k23 = -cam0->cy / cam0->fy;

    cam1->inv_k11 = 1.f / cam1->fx;
    cam1->inv_k22 = 1.f / cam1->fy;
    cam1->inv_k13 = -cam1->cx / cam1->fx;
    cam1->inv_k23 = -cam1->cy / cam1->fy;

    fmt::print("Camera Intrinsic:\n");
    fmt::print("cam0 - fx:{},fy:{},cx:{},cy:{},baseline:{}\n",cam0->fx,cam0->fy,cam0->cx,cam0->cy,cam0->baseline);
    fmt::print("cam1 - fx:{},fy:{},cx:{},cy:{},baseline:{}\n",cam1->fx,cam1->fy,cam1->cx,cam1->cy,cam1->baseline);
    cout<<"Read Camera Intrinsic Finished"<<endl;
}



bool PinHoleCamera::ReadFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened()){
        return false;
    }

    if (!fs["model_type"].isNone()){
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType != "PINHOLE"){
            return false;
        }
    }

    fs["camera_name"] >> camera_name;
    fs["image_width"] >>image_width;
    fs["image_height"] >> image_height;

    cv::FileNode n = fs["distortion_parameters"];
    k1 = static_cast<float>(n["k1"]);
    k2 = static_cast<float>(n["k2"]);
    p1 = static_cast<float>(n["p1"]);
    p2 = static_cast<float>(n["p2"]);

    n = fs["projection_parameters"];
    fx = static_cast<float>(n["fx"]);
    fy = static_cast<float>(n["fy"]);
    cx = static_cast<float>(n["cx"]);
    cy = static_cast<float>(n["cy"]);

    return true;
}


/**
 * 将特征点从图像平面反投影到归一化平面,并去畸变
 * @param p
 * @param P
 */
void PinHoleCamera::LiftProjective(const Vec2d& p, Vec3d& P) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;

    mx_d = inv_k11 * p(0) + inv_k13;
    my_d = inv_k22 * p(1) + inv_k23;

    ///TODO
    if (1){
        mx_u = mx_d;
        my_u = my_d;
    }
    else{
        double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

        // Apply inverse distortion model
        // proposed by Heikkila
        mx2_d = mx_d*mx_d;
        my2_d = my_d*my_d;
        mxy_d = mx_d*my_d;
        rho2_d = mx2_d+my2_d;
        rho4_d = rho2_d*rho2_d;
        radDist_d = k1*rho2_d+k2*rho4_d;
        Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
        Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
        inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

        mx_u = mx_d - inv_denom_d*Dx_d;
        my_u = my_d - inv_denom_d*Dy_d;
    }

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}






