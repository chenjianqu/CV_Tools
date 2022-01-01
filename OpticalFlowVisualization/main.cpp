/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of RAFT_CPP.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/optflow.hpp>

#include <torch/torch.h>

#include "Visualization.h"

int main() {
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

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
