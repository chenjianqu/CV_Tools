//
// Created by chen on 2021/11/30.
//


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

int main(int argc,char **argv)
{
    if(argc!=2){
        cout<<"参数:视频路径"<<endl;
        std::terminate();
    }

    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        throw runtime_error("Cannot open cv::VideoCapture");
    }

    auto image = cv::Mat();
    cv::namedWindow("Output");
    int i=0;
    while (cap.read(image)) {
        //获得帧索引
        auto frame_processed = static_cast<uint32_t>(cap.get(cv::CAP_PROP_POS_FRAMES)) - 1;

        cv::imshow("Output", image);
        cv::waitKey(30);

    }


    cap.release();

    return 0;


}

