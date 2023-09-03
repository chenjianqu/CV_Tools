#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ORBextractor.h"

int main() {

    cv::Mat img = cv::imread("000000.png");
    cv::Mat img_gray;
    cv::cvtColor(img,img_gray,CV_BGR2GRAY);

    ORB_SLAM3::ORBextractor orb_extractor(200,1.2,8,20,7);

    std::vector<int> vLapping = {0,1000};
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    int num = orb_extractor(img_gray,cv::Mat(),keypoints,descriptors,vLapping);

    for(auto &kp:keypoints){
        cv::circle(img,kp.pt,4,cv::Scalar(0,0,255),2);
    }


    cv::imshow("test",img);
    cv::waitKey(0);

    return 0;
}
