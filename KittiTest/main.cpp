#include <iostream>
#include <string>
#include <filesystem>
#include <sstream>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/optflow.hpp>


namespace fs = std::filesystem;
using namespace std;

string kPreprocessFlowPath = "/home/chen/datasets/kitti/tracking/flow_02/training/image_02/0003/";

cv::Mat ReadFlowImage(unsigned int seq_id){
    ///补零
    int name_width=6;
    stringstream ss;
    ss<<std::setw(name_width)<<std::setfill('0')<<seq_id;
    string target_name;
    ss >> target_name;

    ///获取目录中所有的文件名
    static vector<fs::path> names;
    if(names.empty()){
        fs::path dir_path(kPreprocessFlowPath);
        if(!fs::exists(dir_path))
            return {};
        fs::directory_iterator dir_iter(dir_path);
        for(auto &it : dir_iter)
            names.emplace_back(it.path().filename());
        std::sort(names.begin(),names.end());
    }
    for(auto &p : names){
        cout<<p.string()<<endl;
    }

    cv::Mat flow_img;

    ///二分查找
    int low=0,high=names.size()-1;
    while(low<=high){
        int mid=(low+high)/2;
        string name_stem = names[mid].stem().string();
        cout<<low<<" "<<high<<" "<<name_stem<<endl;
        if(name_stem == target_name){
            string n_path = (kPreprocessFlowPath/names[mid]).string();
            flow_img = cv::optflow::readOpticalFlow(n_path);
            break;
        }
        else if(name_stem > target_name){
            high = mid-1;
        }
        else{
            low = mid+1;
        }
    }

    if(flow_img.empty()){
        cerr<<"Can not find the target name:"<<target_name<<" in dir:" << kPreprocessFlowPath<<endl;
        std::terminate();
    }

    return flow_img;
    /*if(read_flow.empty())
        return {};
    torch::Tensor tensor = torch::from_blob(read_flow.data, {read_flow.rows,read_flow.cols ,2}, torch::kFloat32).to(torch::kCUDA);
    tensor = tensor.permute({2,0,1});
    return tensor;*/
}


int main() {
    std::cout << "Hello, World!" << std::endl;

    cv::Mat flow = ReadFlowImage(1);
    cout<<flow.size<<endl;

    return 0;
}
