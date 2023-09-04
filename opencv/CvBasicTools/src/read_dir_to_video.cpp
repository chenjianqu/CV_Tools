//
// Created by cjq on 23-8-23.
//
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace std;
namespace fs = std::filesystem;



void DrawTextWithBackground(cv::Mat &image,const string& text,const cv::Point2i &point_ori,
                            int font_face,double font_scale,const cv::Scalar &color,int thickness){
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    baseline += thickness;
    //draw the box
    rectangle(image, point_ori + cv::Point(0, baseline),
              point_ori + cv::Point(textSize.width, -textSize.height),
              cv::Scalar(255, 255, 255),-1);
    putText(image, text, point_ori, font_face, font_scale, color, thickness, 8);
}



DEFINE_string(write_video_path,"output.mp4","write_video_path");
DEFINE_string(image_dir_path,"","image_dir_path");
DEFINE_bool(show_image_name,false,"show_image_name");

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_image_dir_path.empty());

    std::unique_ptr<cv::VideoWriter> video_writer;

    if(!fs::exists(FLAGS_image_dir_path)){
        SPDLOG_ERROR("In ReadPointCloudFromTxt(), file_dir:{} not exists!",FLAGS_image_dir_path);
        return {};
    }
    if(fs::is_empty(FLAGS_image_dir_path)){
        SPDLOG_ERROR("In ReadPointCloudFromTxt(), file_dir:{} is empty!",FLAGS_image_dir_path);
        return {};
    }

    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.;
    int thickness = 1;
    string text = "test";
    int baseline;
    cv::Size textSize = cv::getTextSize(text, font_face, font_scale, thickness,&baseline);
    int text_height = textSize.height;


    fs::directory_iterator dir(FLAGS_image_dir_path);
    std::vector<fs::path> all_path(begin(dir),end(dir));
    std::sort(all_path.begin(),all_path.end());


    ///遍历目录
    int cnt = 0;
    for (auto& file : all_path) {
        cv::Mat img = cv::imread(file.string());
        if(FLAGS_show_image_name){
            DrawTextWithBackground(img,file.filename().string(),cv::Point2i(30,text_height+20),font_face,
                                   font_scale,cv::Scalar(0,0,255),thickness);
        }

        cnt++;
        double percent = cnt*100. / all_path.size();
        cout<<fmt::format("[{:.2f}%] read:{}",percent,file.string())<<endl;
        if(!FLAGS_write_video_path.empty()) {
            if(!video_writer){
                video_writer = std::make_unique<cv::VideoWriter>(
                        FLAGS_write_video_path, 0x7634706d, 10 ,
                        cv::Size(img.cols, img.rows));
            }
            video_writer->write(img);
        }
    }
    if(video_writer)
        video_writer->release();

    return 0;
}