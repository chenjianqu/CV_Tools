//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/file_utils.h"

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=4){
        std::cerr<<"usage:./add_manual_mask ${mask_dir_path} ${manual_mask_path} ${save_path}"<<std::endl;
        return -1;
    }

    fs::path mask_dir_path(argv[1]);
    fs::path manual_mask_path(argv[2]);
    fs::path save_path(argv[3]);

    if(!fs::exists(mask_dir_path)){
        std::cerr<<mask_dir_path<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!fs::exists(manual_mask_path)){
        std::cerr<<manual_mask_path<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!fs::exists(save_path)){
        std::cerr<<save_path<<" does not exist!"<<std::endl;
        return -1;
    }

    vector<string> img_names;
    GetAllImageNames(mask_dir_path,img_names);

    cv::Mat mask_manual = cv::imread(manual_mask_path.string(),cv::IMREAD_UNCHANGED);

    for(const auto &name:img_names){
        string one_img_path = mask_dir_path / name;

        cv::Mat raw_mask = cv::imread(one_img_path,cv::IMREAD_UNCHANGED);

        //创建一张空白图像
        cv::Mat save_mask = cv::Mat(raw_mask.size(),raw_mask.type(),
                                    cv::Scalar(0));

        int rows = raw_mask.rows;
        int cols = raw_mask.cols;
        for(int i=0;i<rows;++i){
            for(int j=0;j<cols;++j){
                uchar pixel_raw = raw_mask.at<uchar>(i,j);
                uchar pixel_manual = mask_manual.at<uchar>(i,j);
                if(pixel_raw >0 && pixel_manual>0){
                    save_mask.at<uchar>(i,j) = 255;
                }
            }
        }

        string save_img_path=(save_path/name).string();
        cv::imwrite(save_img_path,save_mask);
        cout<<save_img_path<<endl;
    }


    return 0;
}

