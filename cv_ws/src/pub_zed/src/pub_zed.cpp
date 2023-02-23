/*******************************************************
* Copyright (C) 2022, Chen Jianqu, Shanghai University
*
* This file is part of dynamic_vins.
* Github:https://github.com/chenjianqu/dynamic_vins
*
* Licensed under the MIT License;
* you may not use this file except in compliance with the License.
*******************************************************/

#include <iostream>
#include <chrono>
#include<thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "def.h"

using namespace std;
using namespace cv;



const string kCam0Topic="/zed_cam/cam0";
const string kCam1Topic="/zed_cam/cam1";

const int kWidth = 1280;
const int kHeight = 720;
int counter=0;

cv::Rect rect0,rect1;

image_transport::Publisher pub_left,pub_right;


int PubUVC(int argc, char** argv)
{
    if(argc!=3 && argc!=2){
        cout<<"please input cam_index and frame_time(ms)"<<endl;
        return -1;
    }

    int cam_index = stoi(argv[1]);
    int delay_ms = 50;

    if(argc==3){
        delay_ms = stoi(argv[2]);
    }


    cv:: VideoCapture cap = cv::VideoCapture(cam_index);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, kWidth*2);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, kHeight);
    cv::Mat frame;
    cap.read(frame);
    cout<<frame.size<<endl;

    while(cap.isOpened() && ros::ok()){
        auto start = std::chrono::steady_clock::now();

        cap.read(frame);

        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }
        std_msgs::Header header;
        header.stamp=ros::Time::now();
        header.seq=counter;

        cv::Mat img0 = frame(rect0);
        cv::Mat img1 = frame(rect1);
        //ssdf  cout<<frame.rows<<"x"<<frame.cols<<endl;

        sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(header, "bgr8", img0).toImageMsg();
        sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(header, "bgr8", img1).toImageMsg();
        pub_left.publish(msg_left);
        pub_right.publish(msg_right);

        ros::spinOnce();

        int used_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

        if(used_time < delay_ms-2){
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms - used_time));
        }

        cout<<counter<<" "<<used_time<<"ms"<<endl;
        counter++;
    }

    return 0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_stereo_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    rect0 = cv::Rect(0,0,kWidth,kHeight);
    rect1 = cv::Rect(kWidth,0,kWidth,kHeight);

    image_transport::ImageTransport it(n);
    pub_left = it.advertise(kCam0Topic,100);
    pub_right = it.advertise(kCam1Topic,100);

    return PubUVC(argc,argv);
}

