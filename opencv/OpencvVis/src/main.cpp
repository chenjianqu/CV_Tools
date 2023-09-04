//
// Created by lucas on 2020/10/13.
//
//
#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

using namespace std;
using namespace cv;

void Demo(){
    // ----------------------创建窗口----------------------
    viz::Viz3d myWindow("Coordinate Frame");
    //----------------------读入相机位姿----------------------
    Vec3d cam1_pose(0, 0, 0), cam1_focalPoint(0, 0, 1), cam1_y_dir(0, -1, 0); // 设置相机的朝向（光轴朝向）
    Affine3d cam_3_pose = viz::makeCameraPose(cam1_pose, cam1_focalPoint, cam1_y_dir); // 设置相机位置与朝向

    myWindow.showWidget("World_coordinate",viz::WCoordinateSystem(),cam_3_pose); // 创建3号相机位于世界坐标系的原点
    // 创建R\T
    Matx33d PoseR_0,PoseR_1,PoseR_2; // 旋转矩阵
    Vec3d PoseT_0,PoseT_1,PoseT_2; // 平移向量
    PoseR_0 = Matx33d(0.0700133,	0.00325626,	0.997541,0.997535,	0.00439059,	-0.0700273,-0.00460782,	0.999985,	-0.00294083);
    PoseT_0 = Vec3d(-0.128633,0.000609044,0.122929) * 10;
    PoseR_1 = Matx33d(-0.821903,0.0251458,-0.569073,  -0.56962,-0.0416208,0.820854,-0.00304428,0.998817,0.0485318);
    PoseT_1 = Vec3d(0.0754863,-0.108494,  0.113143 ) * 10;
    PoseR_2 = Matx33d(0.880609,0.0740675,-0.468019,-0.469291,-0.000261475,-0.883044, -0.0655272,0.997253,0.034529);
    PoseT_2 = Vec3d(0.0624015,0.109845,0.119439) * 10;

    Affine3d Transpose03(PoseR_0,PoseT_0); // 03相机变换矩阵
    Affine3d Transpose13(PoseR_1,PoseT_1); // 13相机变换矩阵
    Affine3d Transpose23(PoseR_2,PoseT_2); // 23相机变换矩阵
    // ----------------------设置坐标系----------------------
    myWindow.showWidget("Cam0",viz::WCoordinateSystem(),Transpose03);
    myWindow.showWidget("Cam1",viz::WCoordinateSystem(),Transpose13);
    myWindow.showWidget("Cam2",viz::WCoordinateSystem(),Transpose23);
    // ----------------------显示----------------------
    myWindow.spin();
    cout << "Hello World" << endl;
};


Affine3d EigenPoseToVisPose(const Eigen::Vector3d &P,const Eigen::Quaterniond &Q){
    Q.normalized();

    Eigen::Matrix3d tmp_R = Q.matrix();
    Matx33d PoseR_0 = Matx33d(tmp_R(0,0),tmp_R(0,1),tmp_R(0,2),
                              tmp_R(1,0),tmp_R(1,1),tmp_R(1,2),
                              tmp_R(2,0),tmp_R(2,1),tmp_R(2,2)
    );
    Vec3d PoseT_0 = Vec3d(P.x(),P.y(),P.z());

    Affine3d Twc(PoseR_0,PoseT_0);
    return Twc;
}


int main(int argc, char **argv) {

    // ----------------------创建窗口----------------------
    viz::Viz3d myWindow("Coordinate Frame");
    //----------------------读入相机位姿----------------------
    //Vec3d cam1_pose(0, 0, 0), cam1_focalPoint(0, 0, 1), cam1_y_dir(0, -1, 0); // 设置相机的朝向（光轴朝向）
    //Affine3d cam_3_pose = viz::makeCameraPose(cam1_pose, cam1_focalPoint, cam1_y_dir); // 设置相机位置与朝向

    //myWindow.showWidget("World_coordinate",viz::WCoordinateSystem(),cam_3_pose); // 创建3号相机位于世界坐标系的原点

    Eigen::Matrix3d tmp_R;
    Eigen::Vector3d tmp_P;

    Eigen::Vector3d PoseT_0_Eigen(-0.059667,-0.394760,-1.201466);
    Eigen::Quaterniond PoseQ_0_Eigen(0.656049,0.634284,-0.285841,0.292538);
    Affine3d T0 = EigenPoseToVisPose(PoseT_0_Eigen,PoseQ_0_Eigen);

    Eigen::Vector3d PoseT_1_Eigen(-0.061890,-0.150175,-1.208667);
    Eigen::Quaterniond PoseQ_1_Eigen(0.656037,0.634255,-0.286188,0.292289);
    Affine3d T1 = EigenPoseToVisPose(PoseT_1_Eigen,PoseQ_1_Eigen);

    // ----------------------设置坐标系----------------------
    myWindow.showWidget("Cam0",viz::WCoordinateSystem(),T0);
    myWindow.showWidget("Cam1",viz::WCoordinateSystem(),T1);
    // ----------------------显示----------------------
    myWindow.spin();

    return 0;
}

