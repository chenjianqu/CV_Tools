#include <iostream>
#include <fstream>
#include <vector>
#include <random>

#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <spdlog/spdlog.h>

using namespace std;

std::string PadNumber(int number,int name_width){
    std::stringstream ss;
    ss<<std::setw(name_width)<<std::setfill('0')<<number;
    string target_name;
    ss >> target_name;
    return target_name;
}

inline cv::Scalar_<unsigned int> getRandomColor(){
    static std::default_random_engine rde;
    static std::uniform_int_distribution<unsigned int> color_rd(0,255);
    return {color_rd(rde),color_rd(rde),color_rd(rde)};
}

torch::Tensor LoadTensor(const string &load_path){

    std::ifstream input(load_path, std::ios::binary);
    std::vector<char> bytes( (std::istreambuf_iterator<char>(input)),
                             (std::istreambuf_iterator<char>()));
    input.close();

    torch::IValue x = torch::pickle_load(bytes);
    torch::Tensor tensor = x.toTensor();
    return tensor;
}


cv::Mat VisualTensor(torch::Tensor &seg_label)
{
    int cols = seg_label.size(2);
    int rows = seg_label.size(1);
    int num_mask = seg_label.size(0);

    auto mask_size=cv::Size(cols ,rows);
    auto mask_tensor = seg_label.to(torch::kInt8).abs().clamp(0,1);

    ///计算合并的mask
    auto merge_tensor = (mask_tensor.sum(0).clamp(0,1)*255).to(torch::kUInt8).to(torch::kCPU);
    auto mask = cv::Mat(mask_size,CV_8UC1,merge_tensor.data_ptr()).clone();
    cv::cvtColor(mask,mask,CV_GRAY2BGR);

    //auto color = getRandomColor();

    return mask;
}



int main() {
    int index=0;

    string base_path = "/home/chen/datasets/kitti/tracking/det2d_02/training/image_02/0002/";

    while(true){
        std::string seq_str = PadNumber(index,6);

        torch::Tensor seg_label = LoadTensor(base_path + fmt::format("seg_label_{}.pt",seq_str));
        torch::Tensor cate_score = LoadTensor(base_path + fmt::format("cate_score_{}.pt",seq_str));
        torch::Tensor cate_label = LoadTensor(base_path + fmt::format("cate_label_{}.pt",seq_str));

        cv::Mat mask_img = VisualTensor(seg_label);

        cv::imshow("mask_img", mask_img);
        cv::waitKey(1);

        /*if(auto order=(cv::waitKey(1) & 0xFF); order == 'q')
            break;
        else if(order==' ')
            cv::waitKey(0);*/

        index++;
    }

    return 0;
}
