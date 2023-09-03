#include <iostream>
#include <chrono>
#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

bool CheckIsDir(const string &dir) {
    if (! std::filesystem::exists(dir)) {
        cout<<dir<<" not exists. Please check."<<endl;
        return false;
    }
    std::filesystem::directory_entry entry(dir);
    if (entry.is_directory())
        return true;
    return false;
}



void GetAllImageFiles(const string& dir, vector<string> &files) {
    // 首先检查目录是否为空，以及是否是目录
    if (!CheckIsDir(dir))
        return;

    // 递归遍历所有的文件
    std::filesystem::directory_iterator iters(dir);
    for(auto &iter: iters) {
        string file_path(dir);
        file_path += "/";
        file_path += iter.path().filename();

        // 查看是否是目录，如果是目录则循环递归
        if (CheckIsDir(file_path)) {
            GetAllImageFiles(file_path, files);
        }
        //不是目录则检查后缀是否是图像
        else {
            string extension = iter.path().extension(); // 获取文件的后缀名
            if (extension == ".jpg" || extension == ".png" || extension == ".jpeg" || extension==".bmp") {
                files.push_back(file_path);
            }
        }
    }
}


void SplitStereo(){

    const string stereo_path="/home/chen/datasets/MyData/calib/calib_1280x720/raw";
    const string save_left_path="/home/chen/datasets/MyData/calib/calib_1280x720/left";
    const string save_right_path="/home/chen/datasets/MyData/calib/calib_1280x720/right";

    constexpr int Width = 1280;
    constexpr int Height = 720;

    cv::Rect rect0(0,0,Width,Height);
    cv::Rect rect1(Width,0,Width,Height);

    vector<string> stereo_files;
    GetAllImageFiles(stereo_path,stereo_files);
    std::sort(stereo_files.begin(),stereo_files.end());

    for(int i=0;i<stereo_files.size();++i){
        cv::Mat stereo_img = cv::imread(stereo_files[i]);
        cout<<stereo_img.size<<endl;
        cv::Mat img0 = stereo_img(rect0);
        cv::Mat img1 = stereo_img(rect1);

        std::filesystem::path fp(stereo_files[i]);

        cv::imwrite(save_left_path+"/" + fp.stem().string()+".png",img0);
        cv::imwrite(save_right_path+"/" + fp.stem().string()+".png",img1);

        cv::imshow("left",img0);
        cv::waitKey(1);

        cout<<i<<endl;
    }

}



int main(int argc, char** argv) {

    SplitStereo();

    return 0;
}
