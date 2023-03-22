#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <spdlog/spdlog.h>

#include "dataloader.h"
#include "image_viewer.h"

namespace fs=std::filesystem;

inline cv::Scalar_<unsigned int> getRandomColor(){
    static std::default_random_engine rde;
    static std::uniform_int_distribution<unsigned int> color_rd(0,255);
    return {color_rd(rde),color_rd(rde),color_rd(rde)};
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


std::tuple<torch::Tensor,torch::Tensor,torch::Tensor> LoadMaskTensor(string seq_id,const fs::path &mask_dir){
    fs::path seg_label_path = mask_dir / fmt::format("seg_label_{}.pt",seq_id);
    if(!fs::exists(seg_label_path)){
        cerr<<seg_label_path<<" does not exist"<<endl;
        return {};
    }


    torch::Tensor seg_label = LoadTensor(mask_dir / fmt::format("seg_label_{}.pt",seq_id));
    torch::Tensor cate_score = LoadTensor(mask_dir / fmt::format("cate_score_{}.pt",seq_id));
    torch::Tensor cate_label = LoadTensor(mask_dir / fmt::format("cate_label_{}.pt",seq_id));
    return {seg_label,cate_score,cate_label};
}


int main(int argc,char** argv) {

    if(argc!=3){
        cerr<<"usage: ./VisualizeInstanceMask ${img_dir} ${mask_dir}"<<endl;
        return -1;
    }

    fs::path img_dir(argv[1]);
    fs::path mask_dir(argv[2]);

    Dataloader dataloader(img_dir);
    ImageViewer imageViewer;

    TicToc t_all,t_step;

    while(true){
        t_all.Tic();
        t_step.Tic();

        cv::Mat color = dataloader.LoadMonoImages();
        cout<<"LoadMonoImages:"<<t_step.TocThenTic()<<" ms"<<endl;

        if(color.empty()){
            break;
        }

        auto [seg_label,cate_score,cate_label] = LoadMaskTensor(dataloader.get_seq_id(),mask_dir);
        cout<<"LoadMaskTensor:"<<t_step.TocThenTic()<<" ms"<<endl;

        cv::Mat img_show;

        if(seg_label.defined()){
            cv::Mat mask_img = VisualTensor(seg_label);
            cv::scaleAdd(mask_img,0.8,color,img_show);
        }
        else{
            img_show = color;
        }

        cout<<"VisualTensor && scaleAdd："<<t_step.TocThenTic()<<" ms"<<endl;

        imageViewer.ImageShow(img_show,30);

        cout<<"T_all:"<<t_all.Toc()<<" ms"<<endl<<endl;
    }

    return 0;
}
