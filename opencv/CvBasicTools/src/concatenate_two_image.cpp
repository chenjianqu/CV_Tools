//
// Created by cjq on 23-6-2.
//
#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

using namespace std;


void CvNormTest(){
    cv::Mat arr(2,2,CV_32F);
    arr.at<float>(0,0) = 1;
    arr.at<float>(0,1) = 2;
    arr.at<float>(1,0) = 3;
    arr.at<float>(1,1) = 4;

    cv::normalize(arr,arr);

    for(int r=0;r<arr.rows;++r){
        for(int c=0;c<arr.cols;++c) {
            cout<<arr.at<float>(r,c)<<" ";
        }
        cout<<endl;
    }

}


int main(int argc,char** argv) {

    CvNormTest();

    if(argc!=4){
        cerr<<"Usage:./concatenate_two_image ${image_path_1} ${image_path_2} ${output+image_path}"<<endl;
        return -1;
    }

    fs::path image_path_1(argv[1]);
    fs::path image_path_2(argv[2]);
    fs::path output_image_path(argv[3]);

    cv::Mat img1 = cv::imread(image_path_1.string());
    cv::Mat img2 = cv::imread(image_path_2.string());

    vector<cv::Mat>vImgs;
    cv::Mat result;
    vImgs.push_back(img1);
    vImgs.push_back(img2);
    //cv::vconcat(vImgs, result); //垂直方向拼接
    cv::hconcat(vImgs, result);

    cv::imwrite(output_image_path.string(),result);


    return 0;
}

