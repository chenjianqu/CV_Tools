//
// Created by chen on 2022/10/30.
//


#include <iostream>
#include <memory>
#include <string>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "camodocal/camera_models/CameraFactory.h"


#include "log_utils.h"
#include "camera_model.h"

int main(){
    string config_path= "/home/chen/ws/dynamic_ws/src/dynamic_vins/config/euroc/euroc.yaml";

    MyLogger::InitLogger(config_path);

    //InitCamera(config_path);

    string calib_file = "/home/chen/CLionProjects/PL-VINS_ws/src/PL-VINS/config/euroc/loop.yaml";

    camodocal::CameraPtr m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

    cv::Mat undist_map1_, undist_map2_ ;

    cv::Mat K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);

    cout<<CvMatToStr<float>(K_)<<endl;

    return 0;
}




