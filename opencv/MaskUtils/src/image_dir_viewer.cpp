//
// Created by cjq on 23-4-18.
//

#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/file_utils.h"
#include "utils/image_viewer.h"

namespace fs=std::filesystem;

int main(int argc,char** argv) {
    if(argc!=2){
        std::cerr<<"usage:./image_dir_viewer ${image_dir}"<<std::endl;
        return -1;
    }

    fs::path image_dir(argv[1]);

    if(!fs::exists(image_dir)){
        std::cerr<<image_dir<<" does not exist!"<<std::endl;
        return -1;
    }

    vector<string> image_names;
    GetAllImageNames(image_dir,image_names);
    std::sort(image_names.begin(),image_names.end());

    ImageViewer viewer;

    for(int i=0;i<image_names.size();++i){
        string image_single_path = image_dir / image_names[i];
        cout<<image_single_path<<endl;

        cv::Mat img = cv::imread(image_single_path,cv::IMREAD_UNCHANGED);

        if(img.empty()){
            cerr<<"Can not open:"<<image_single_path<<endl;
        }
        viewer.ImageShow(img,50);
    }

    return 0;
}

