//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=3){
        std::cerr<<"usage:./visualze_mask_class ${img_path} ${class_id}"<<std::endl;
        return -1;
    }

    fs::path img_path(argv[1]);

    int class_id = std::stoi(argv[2]);

    if(!fs::exists(img_path)){
        std::cerr<<img_path<<" does not exist!"<<std::endl;
        return -1;
    }

    cv::Mat mask = cv::imread(img_path.string(),cv::IMREAD_UNCHANGED);
    int rows = mask.rows;
    int cols = mask.cols;

    std::cout<<"rows:"<<rows<<" cols:"<<cols<<std::endl;
    std::cout<<"type:"<<mask.type()<<std::endl;

    cv::Mat class_mask = cv::Mat(mask.size(),mask.type(),cv::Scalar(0));

    for(int i=0;i<rows;++i){
        for(int j=0;j<cols;++j){
            if(mask.at<uchar>(i,j)==class_id){
                class_mask.at<uchar>(i,j) = 255;
            }
        }
    }

    cv::imshow("class_mask",class_mask);
    cv::waitKey(0);

    return 0;
}

