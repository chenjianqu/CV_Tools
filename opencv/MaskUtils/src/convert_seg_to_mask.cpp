//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/def.h"
#include "utils/file_utils.h"

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=4){
        std::cerr<<"usage:./convert_seg_to_mask ${img_path} ${save_path} ${mask_class_ids_str}"<<std::endl;
        std::cerr<<"where the ${mask_class_ids_str} should be \"a c b\" style"<<std::endl;
        return -1;
    }

    fs::path img_path(argv[1]);
    fs::path save_path(argv[2]);
    string mask_class_ids_str=argv[3];

    if(!fs::exists(img_path)){
        std::cerr<<img_path<<" does not exist!"<<std::endl;
        return -1;
    }

    vector<string> tokens;
    split(mask_class_ids_str,tokens," ");

    ///设置需要掩玛的类别
    vector<int> class_ids(256,1);

    for(string &token : tokens){
        int class_id = std::stoi(token);
        if(class_id>=0 && class_id<=255){
            class_ids[class_id]=0;
        }
        else{
            cerr<<"invalid class_id"<<endl;
            return -1;
        }
    }

    vector<string> img_names;
    GetAllImageNames(img_path,img_names);

    for(const auto &name:img_names){
        string one_img_path = img_path / name;

        cv::Mat raw_mask = cv::imread(one_img_path,cv::IMREAD_UNCHANGED);

        //创建一张空白图像
        cv::Mat save_mask = cv::Mat(raw_mask.size(),raw_mask.type(),
                                    cv::Scalar(255));

        int rows = raw_mask.rows;
        int cols = raw_mask.cols;
        for(int i=0;i<rows;++i){
            for(int j=0;j<cols;++j){
                uchar pixel = raw_mask.at<uchar>(i,j);
                if(class_ids[pixel] == 0){
                    save_mask.at<uchar>(i,j) = 0;
                }
            }
        }

        string save_img_path=(save_path/name).string();
        cv::imwrite(save_img_path,save_mask);
        cout<<save_img_path<<endl;
    }


    return 0;
}

