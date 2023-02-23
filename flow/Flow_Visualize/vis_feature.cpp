/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of RAFT_CPP.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <iostream>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/optflow.hpp>

#include <torch/torch.h>

#include "Visualization.h"

using std::vector;
using std::string;
namespace fs = std::filesystem;

using namespace std;

void VisOneImage(){
    cv::Mat imFlow = cv::optflow::readOpticalFlow("../000000.flo");
    if(imFlow.empty()){
        std::cerr<<"open flow failed"<<std::endl;
    }

    torch::Tensor tensor = torch::from_blob(imFlow.data, {imFlow.rows,imFlow.cols ,2}, torch::kFloat32).to(torch::kCUDA);
    tensor = tensor.permute({2,0,1});

    auto show = visual_flow_image(tensor);


    cv::imwrite("output.jpg",show);
    cv::imshow("flow",show);
    cv::waitKey(0);
}


vector<fs::path> ReadImagesNames(const string &path){
    fs::path dir_path(path);
    vector<fs::path> names;
    if(!fs::exists(dir_path))
        return names;
    fs::directory_iterator dir_iter(dir_path);
    for(auto &it : dir_iter){
        names.push_back(it.path());
    }
    return names;
}


void PrintFlow(){
    cv::Mat imFlow = cv::optflow::readOpticalFlow("/home/chen/temp/flow0/1597198389.276270_cam0.png");
    if(imFlow.empty()){
        std::cerr<<"open flow failed"<<std::endl;
    }

    int cnt=0;
    for(int i=0;i<imFlow.rows;++i){
        for(int j=0;j<imFlow.cols;++j){
            if(i==j){
                auto p=imFlow.at<cv::Vec2f>(i,j);
                cout<<"("<<p[0]<<","<<p[1]<<") ";
                cnt++;
                if(cnt%10 == 0)
                    cout<<endl;
            }
        }
    }

    cv::waitKey(0);
}


void VisFlowDataset(){
    const string &dir = "/home/chen/datasets/kitti/tracking/flow_02/training/image_02/0003";
    auto names = ReadImagesNames(dir);
    std::sort(names.begin(),names.end());

    for(int i=0;i<names.size();++i){
        cv::Mat imFlow = cv::optflow::readOpticalFlow(names[i].string());
        if(imFlow.empty()){
            std::cerr<<"open flow failed"<<std::endl;
            break;
        }
        torch::Tensor tensor = torch::from_blob(imFlow.data, {imFlow.rows,imFlow.cols ,2}, torch::kFloat32).to(torch::kCUDA);
        tensor = tensor.permute({2,0,1});
        auto show = visual_flow_image(tensor);
        cv::imwrite("output.jpg",show);
        cv::imshow("flow",show);
        cv::waitKey(1);
    }

}



int main() {
    VisFlowDataset();
    //PrintFlow();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
