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
    if(argc!=3 && argc!=4){
        std::cerr<<"usage:./mask_rgb_vis ${mask_dir} ${rgb_dir} ${save_path}"<<std::endl;
        std::cerr<<"or"<<std::endl;
        std::cerr<<"usage:./mask_rgb_vis ${mask_dir} ${rgb_dir}"<<std::endl;
        return -1;
    }


    fs::path mask_dir(argv[1]);
    fs::path rgb_dir(argv[2]);

    fs::path save_path;
    if(argc==4){
        save_path=argv[3];
    }

    if(!fs::exists(mask_dir)){
        std::cerr<<mask_dir<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!fs::exists(rgb_dir)){
        std::cerr<<rgb_dir<<" does not exist!"<<std::endl;
        return -1;
    }
    if(!save_path.empty() && !fs::exists(save_path)){
        std::cerr<<save_path<<" does not exist!"<<std::endl;
        return -1;
    }

    vector<string> mask_names;
    GetAllImageNames(mask_dir,mask_names);
    vector<string> rgb_names;
    GetAllImageNames(rgb_dir,rgb_names);
    std::sort(mask_names.begin(),mask_names.end());
    std::sort(rgb_names.begin(),rgb_names.end());

    if(mask_names.size() != rgb_names.size()){
        cerr<<"mask_names.size() != rgb_names.size()"<<endl;
        return -1;
    }

    ImageViewer viewer;

    for(int i=0;i<mask_names.size();++i){
        string mask_single_path = mask_dir / mask_names[i];
        string rgb_single_path = rgb_dir / rgb_names[i];

        cout<<mask_single_path<<endl;
        cout<<rgb_single_path<<endl;

        cv::Mat mask = cv::imread(mask_single_path,0);
        cv::Mat rgb = cv::imread(rgb_single_path,1);

        if(mask.empty()){
            cerr<<"Can not open:"<<mask_single_path<<endl;
        }
        if(rgb.empty()){
            cerr<<"Can not open:"<<rgb_single_path<<endl;
        }

        cv::cvtColor(mask,mask,cv::COLOR_GRAY2BGR);

        cv::Mat img_vis;
        cv::scaleAdd(mask,0.4,rgb,img_vis);

        if(!save_path.empty()){
            string save_img_path=(save_path/rgb_names[i]).string();
            cv::imwrite(save_img_path,img_vis);
        }

        viewer.ImageShow(img_vis,30);
    }


    return 0;
}

