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


void VisFlowDataset(){
    const string &dir = "/home/chen/temp/flow0/";
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

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
