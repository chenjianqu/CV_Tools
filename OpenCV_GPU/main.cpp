#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <torch/torch.h>
#include <torchvision/vision.h>

#include <cuda_runtime_api.h>

using namespace std;

class TicToc{
public:
    TicToc(){
        tic();
    }

    void tic(){
        start = std::chrono::system_clock::now();
    }

    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

    void toc_print_tic(const char* str){
        printf("%s : %f ms\n",str,toc());
        tic();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


int main() {
    std::cout << "Hello, World!" << std::endl;

    std::string path = "/media/chen/EC4A17F64A17BBF0/datasets/kitti/raw/2011_09_30/2011_09_30_drive_0027_sync";
    std::string img0_dir_path = path+"/image_02/data/";
    TicToc ticToc;

    cv::Mat img_prev;

    for(int index=0;index<1000;++index)
    {
        char name[64];
        sprintf(name,"%010d.png",index);
        std::string img0_path=img0_dir_path+name;
        cout<<"Read Image:"<<img0_path<<endl;
        cv::Mat img_curr_color=cv::imread(img0_path);
        if(img_curr_color.empty()){
            cout<<"Read:"<<img0_path<<" failure"<<endl;
            break;
        }
        cv::Mat img_curr;
        cv::cvtColor(img_curr_color, img_curr, CV_BGR2GRAY);

        if(index==0){
            img_prev = img_curr;
            continue;
        }

        ticToc.tic();

        cout<<"img_curr "<<img_curr.size<<endl;

        torch::Tensor img_curr_tensor=torch::from_blob(img_curr.data,{img_curr.rows,img_curr.cols},torch::kInt8);
        cout<<"img_curr_tensor "<<img_curr_tensor.sizes()<<endl;

        torch::Tensor img_curr_t_gpu=img_curr_tensor.to(torch::kFloat).to(torch::kCUDA);

        img_curr_t_gpu = img_curr_t_gpu.to(torch::kInt8);

        cv::cuda::GpuMat gpu_mat(cv::Size(img_curr_t_gpu.sizes()[1],img_curr_t_gpu.sizes()[0]),CV_8UC1,img_curr_t_gpu.data_ptr());
        cout<<"gpu_mat "<<gpu_mat.size()<<endl;

        cv::Mat new_imshow = cv::Mat(cv::Size(img_curr_t_gpu.sizes()[1],img_curr_t_gpu.sizes()[0]), CV_8UC1, img_curr_tensor.to(torch::kCPU).data_ptr()).clone();
        cv::imshow("new_imshow",new_imshow);


        cv::Mat img_show;
        gpu_mat.download(img_show);
        cv::imshow("test",img_show);
        cv::waitKey(0);

        break;

    }

    return 0;
}
