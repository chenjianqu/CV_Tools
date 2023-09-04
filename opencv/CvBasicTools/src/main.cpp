#include <iostream>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

using namespace std;

int main() {

    cv::Mat img = cv::imread("/media/cjq/新加卷/datasets/220Dataset/HS5_220_8M/12_bev/20230727174840.191/sfm/seg_raw/1690451192.194_seg.png");

    cout<<fmt::format("size:{} dim:{}",img.size,img.dims)<<endl;

    cout<<img.type()<<endl;

    std::set<std::array<int,3>> class_set;

    for(int r=0;r<img.rows;++r){
        for(int c=0;c<img.cols;++c) {
            auto pixel = img.at<cv::Vec3b>(r,c);
            if(pixel[0]>0 || pixel[1]>0 || pixel[2]>0){
                //cout<<pixel<<" ";
                class_set.insert({pixel[0],pixel[1],pixel[2]});
            }
        }
    }

    for(auto &p:class_set){
        cout<<fmt::format("{} {} {}",p[0],p[1],p[2])<<endl;
    }

    return 0;
}
