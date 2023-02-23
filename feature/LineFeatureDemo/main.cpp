#include <iostream>

#include <opencv2/opencv.hpp>

#include "linefeature_tracker.h"

int main() {
    LineFeatureTracker trackerData;

    cv::Mat img_raw=cv::imread("/home/chen/CLionProjects/CV_Tools/LineFeatureTest/demo/000000.png");
    cout<<img_raw.size<<endl;
    //cv::imshow("img_raw",img_raw);
    //cv::waitKey(0);
    trackerData.readImage(img_raw);   // rowRange(i,j) 取图像的i～j行

    auto un_lines = trackerData.curframe_->vecLine;
    auto &ids = trackerData.curframe_->lineID;

    cv::Mat show=img_raw;
    for (unsigned int j = 0; j < ids.size(); j++){
        cv::line(show,un_lines[j].StartPt,un_lines[j].EndPt,cv::Scalar(255),2);
    }
    cv::imwrite("result1.jpg",show);

    //获得第一张图像的ID
    int N_id=ids.back();

    cv::Mat img1=cv::imread("/home/chen/CLionProjects/CV_Tools/LineFeatureTest/demo/000001.png");

    trackerData.readImage(img1);
    auto un_lines1 = trackerData.curframe_->vecLine;
    auto &ids1 = trackerData.curframe_->lineID;

    show=img1;
    for (unsigned int j = 0; j < ids1.size(); j++){
        if(ids1[j] <=N_id){
            cv::line(show,un_lines1[j].StartPt,un_lines1[j].EndPt,cv::Scalar(255),2);
        }
        else{
            cv::line(show,un_lines1[j].StartPt,un_lines1[j].EndPt,cv::Scalar(0,0,255),2);
        }
    }
    cv::imwrite("result2.jpg",show);

    return 0;
}
