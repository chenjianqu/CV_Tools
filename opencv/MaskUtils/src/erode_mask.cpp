//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/file_utils.h"

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=3){
        std::cerr<<"usage:./erode_mask ${mask_dir_path} ${save_path}"<<std::endl;
        return -1;
    }

    fs::path mask_dir_path(argv[1]);
    fs::path save_path(argv[2]);

    if(!fs::exists(mask_dir_path)){
        std::cerr<<mask_dir_path<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!fs::exists(save_path)){
        std::cerr<<save_path<<" does not exist!"<<std::endl;
        return -1;
    }

    vector<string> img_names;
    GetAllImageNames(mask_dir_path,img_names);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));

    for(const auto &name:img_names){
        string one_img_path = mask_dir_path / name;

        cv::Mat raw_mask = cv::imread(one_img_path,cv::IMREAD_UNCHANGED);

        cv::Mat save_mask;
        cv::erode(raw_mask, save_mask, element);

        string save_img_path=(save_path/name).string();
        cv::imwrite(save_img_path,save_mask);
        cout<<save_img_path<<endl;
    }

    return 0;
}

