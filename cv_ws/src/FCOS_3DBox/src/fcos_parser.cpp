#include <cstdio>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <list>
#include <thread>
#include <regex>
#include<filesystem>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <spdlog/formatter.h>


#include "visualization.h"
#include "utils.h"

using namespace std;
namespace fs=std::filesystem;




vector<fs::path> ReadNames(const string &data_path){
    ///获取目录中所有的文件名
     vector<fs::path> names;
    if(names.empty()){
        fs::path dir_path(data_path);
        if(!fs::exists(dir_path))
            return {};
        fs::directory_iterator dir_iter(dir_path);
        for(auto &it : dir_iter)
            names.emplace_back(it.path().filename());
        std::sort(names.begin(),names.end());
    }
    return names;
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");//防止中文乱码
    ros::init(argc, argv, "kitti_pub");
    ros::start();

    ros::NodeHandle nh;

    ros::Publisher obj_pub=nh.advertise<visualization_msgs::MarkerArray>("fcos_3d_box",10);

    string data_path="/home/chen/CLionProjects/CV_Tools/cv_ws/src/FCOS_3DBox/demo/0002/";

    Eigen::Matrix<double,8,3> corners_norm;
    corners_norm << 0,0,0,  0,0,1,  0,1,1,  0,1,0,  1,0,0,  1,0,1,  1,1,1,  1,1,0;

    Eigen::Vector3d offset(0.5,1,0.5);//预测结果所在的坐标系与相机坐标系之间的偏移
    corners_norm = corners_norm.array().rowwise() - offset.transpose().array();//将每个坐标减去偏移量
    cout<<"corners_norm\n"<<corners_norm<<endl;

    ///获取所有的文件名
    auto names = ReadNames(data_path);
    int Num = names.size();
    int idx=0;

    while(ros::ok())
    {
        visualization_msgs::MarkerArray markers;

        ifstream fp(data_path + names[idx].string());
        idx = (idx+1)%Num;
        string line;
        int index=0;
        while (getline(fp,line)){ //循环读取每行数据
            vector<string> tokens;
            split(line,tokens," ");

            ///每行的前3个数字是类别,属性,分数
            int class_id = std::stoi(tokens[0]);
            int attribution_id = std::stoi(tokens[1]);
            double score = std::stod(tokens[2]);

            if(score<0.15){
                continue;
            }

            string text = fmt::format("class:{} attr:{} score:{}\n",class_id,attribution_id,score);
            fmt::print(text);

            ///3-5个数字是物体包围框底部的中心
            Eigen::Vector3d bottom_center;
            bottom_center<<std::stod(tokens[3]),std::stod(tokens[4]),std::stod(tokens[5]);
            ///6-8数字是物体在x,y,z轴上的大小
            Eigen::Vector3d dims;
            dims<<std::stod(tokens[6]),std::stod(tokens[7]),std::stod(tokens[8]);
            ///9个yaw角(绕着y轴,因为y轴是垂直向下的)
            double yaw = std::stod(tokens[9]);

            Eigen::Matrix<double,3,8> corners = corners_norm.transpose(); //得到矩阵 3x8
            corners = corners.array().colwise() * dims.array();//广播逐点乘法

            cout<<corners<<endl;

            ///根据yaw角构造旋转矩阵
            Eigen::Matrix3d R;
            R<<cos(yaw),0, -sin(yaw),   0,1,0,   sin(yaw),0,cos(yaw);

            Eigen::Matrix<double,8,3> result =  corners.transpose() * R;//8x3
            //加上偏移量
            Eigen::Matrix<double,8,3> output= result.array().rowwise() + bottom_center.transpose().array();

            cout<<"output"<<endl;
            cout<<output<<endl;

            ///计算包围框中心坐标
            Eigen::Vector3d center = (output.row(0)+output.row(6)).transpose()  /2;
            auto center_marker =  BuildSphereMarker(center,0.1,index+1000);
            markers.markers.push_back(center_marker);

            auto cube_marker = BuildLineStripMarker(output,index);
            markers.markers.push_back(cube_marker);

            auto text_marker = BuildTextMarker(text,output.row(0).transpose(),index+2000);
            markers.markers.push_back(text_marker);

            index++;
        }
        fp.close();

        //BuildLineStripMarker(lineStripMarker);
        //BuildTextMarker(textMarker);
        //markers.markers.push_back(textMarker);

        obj_pub.publish(markers);
        ros::spinOnce();

        std::this_thread::sleep_for(300ms);
    }


    return 0;
}