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


DEFINE_string(image_dir_path0,"","image_dir_path0");
DEFINE_string(image_dir_path1,"","image_dir_path1");
DEFINE_string(write_video_path,"output.mp4","write_video_path");
DEFINE_bool(show_image_name,false,"show_image_name");


int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_image_dir_path0.empty());

    std::unique_ptr<cv::VideoWriter> video_writer;

    if(!fs::exists(FLAGS_image_dir_path0)){
        SPDLOG_ERROR("In ReadPointCloudFromTxt(), file_dir:{} not exists!",FLAGS_image_dir_path0);
        return {};
    }
    if(fs::is_empty(FLAGS_image_dir_path1)){
        SPDLOG_ERROR("In ReadPointCloudFromTxt(), file_dir:{} is empty!",FLAGS_image_dir_path1);
        return {};
    }

    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.;
    int thickness = 1;
    string text = "test";
    int baseline;
    cv::Size textSize = cv::getTextSize(text, font_face, font_scale, thickness,&baseline);
    int text_height = textSize.height;


    fs::directory_iterator dir0(FLAGS_image_dir_path0);
    std::vector<fs::path> all_dir0_path(begin(dir0),end(dir0));
    std::sort(all_dir0_path.begin(),all_dir0_path.end());
    int num_path0 = all_dir0_path.size();

    fs::path dir1_path(FLAGS_image_dir_path1);


    ///遍历目录
    int cnt = 0;
    for (auto& file : all_dir0_path) {
        cv::Mat img0 = cv::imread(file.string());
        if(FLAGS_show_image_name){
            DrawTextWithBackground(img0, "dir0_" + file.filename().string(), cv::Point2i(30, text_height + 20),
                                   font_face, font_scale, cv::Scalar(0,0,255), thickness);
        }

        cv::Mat img1;
        fs::path file1 = dir1_path / file.filename();
        if(fs::exists(file1)){
            img1 = cv::imread(file1.string());
            CHECK_EQ(img0.rows, img1.rows);
            CHECK_EQ(img0.cols, img1.cols);

            DrawTextWithBackground(img1,"dir1_"+file.filename().string(),cv::Point2i(30,text_height+20),
                                   font_face,font_scale,cv::Scalar(0,0,255),thickness);
        }
        else{
            img1 = cv::Mat(img0.rows, img0.cols, img0.type(), cv::Scalar(255, 255, 255));
        }

        cv::Mat img_con;
        cv::vconcat(img0, img1, img_con);

        cnt++;
        double percent = cnt*100. / num_path0;
        cout<<fmt::format("[{:.2f}%] read:{}",percent,file.string())<<endl;
        if(!FLAGS_write_video_path.empty()) {
            if(!video_writer){
                video_writer = std::make_unique<cv::VideoWriter>(
                        FLAGS_write_video_path, 0x7634706d, 10 ,
                        cv::Size(img_con.cols, img_con.rows));
            }
            video_writer->write(img_con);
        }
    }
    if(video_writer)
        video_writer->release();

    return 0;
}