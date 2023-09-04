//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=2){
        std::cerr<<"usage:./read_image_diagonal_pixels ${img_path}"<<std::endl;
        return -1;
    }

    fs::path img_path(argv[1]);

    if(!fs::exists(img_path)){
        std::cerr<<img_path<<" does not exist!"<<std::endl;
        return -1;
    }

    cv::Mat img = cv::imread(img_path.string(),cv::IMREAD_UNCHANGED);
    int rows = img.rows;
    int cols = img.cols;

    std::cout<<"rows:"<<rows<<" cols:"<<cols<<std::endl;
    std::cout<<"type:"<<img.type()<<std::endl;

    for(int i=0;i<rows;++i){
        for(int j=0;j<cols;++j){
            if(i==j){
                std::cout<<"<"<<i<<" "<<j<<" "<<(int)img.at<uchar>(i,j)<<">  ";
            }
        }
        if(i%10==0){
            std::cout<<std::endl;
        }
    }


    return 0;
}

