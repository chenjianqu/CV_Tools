//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/file_utils.h"

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=5){
        std::cerr<<"usage:./mask_operation ${mask0_dir} ${mask1_dir} ${mask_operation} ${save_path}"<<std::endl;
        return -1;
    }

    fs::path mask0_dir(argv[1]);
    fs::path mask1_dir(argv[2]);
    string mask_operation(argv[3]);
    fs::path save_path(argv[4]);

    if(!fs::exists(mask0_dir)){
        std::cerr<<mask0_dir<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!fs::exists(mask1_dir)){
        std::cerr<<mask1_dir<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!fs::exists(save_path)){
        std::cerr<<save_path<<" does not exist!"<<std::endl;
        return -1;
    }

    vector<string> mask0_names;
    GetAllImageNames(mask0_dir,mask0_names);
    vector<string> mask1_names;
    GetAllImageNames(mask1_dir,mask1_names);

    if(mask0_names.size() != mask1_names.size()){
        cerr<<"mask0_names.size() != mask1_names.size()"<<endl;
        return -1;
    }

    int operation_flag=0;//与运算
    if(mask_operation=="and"){
        operation_flag=0;
    }
    else if(mask_operation=="or"){
        operation_flag=1;
    }
    else{
        cerr<<mask_operation<<" operation is not implemented"<<endl;
        return -1;
    }

    for(const auto &name:mask0_names){
        string mask0_single_path = mask0_dir / name;
        string mask1_single_path = mask1_dir / name;

        cv::Mat mask0 = cv::imread(mask0_single_path,cv::IMREAD_UNCHANGED);
        cv::Mat mask1 = cv::imread(mask1_single_path,cv::IMREAD_UNCHANGED);

        //创建一张空白图像
        cv::Mat save_mask = cv::Mat(mask0.size(),mask0.type(),cv::Scalar(0));

        int rows = save_mask.rows;
        int cols = save_mask.cols;
        for(int i=0;i<rows;++i){
            for(int j=0;j<cols;++j){
                uchar p0 = mask0.at<uchar>(i,j);
                uchar p1 = mask1.at<uchar>(i,j);

                if(operation_flag==0){
                    if(p0>0 && p1>0){
                        save_mask.at<uchar>(i,j) = 255;
                    }
                }
                else if(operation_flag==1){
                    if(p0>0 || p1>0){
                        save_mask.at<uchar>(i,j) = 255;
                    }
                }

            }
        }

        string save_img_path=(save_path/name).string();
        cv::imwrite(save_img_path,save_mask);
        cout<<save_img_path<<endl;
    }


    return 0;
}

