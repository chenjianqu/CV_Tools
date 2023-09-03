#include <iostream>
#include <chrono>
#include <string>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>




using namespace std;
using namespace cv;

inline std::string PadNumber(int number,int name_width){
    std::stringstream ss;
    ss<<std::setw(name_width)<<std::setfill('0')<<number;
    string target_name;
    ss >> target_name;
    return target_name;
}

void VideoSplit()
{
    const string video_path="/home/chen/datasets/MyData/stereo/videos/2022-12-4.avi";
    const string save_left_path="/home/chen/datasets/MyData/stereo/videos/2022-12-4_left/room";
    const string save_right_path="/home/chen/datasets/MyData/stereo/videos/2022-12-4_right/room";

    constexpr int Width = 1280;
    constexpr int Height = 720;

    cv:: VideoCapture cap = cv::VideoCapture(video_path);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, Width*2);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);
    cv::Mat frame, img;
    cap.read(frame);
    cout<<frame.size<<endl;

    cv::Rect rect0(0,0,Width,Height);
    cv::Rect rect1(Width,0,Width,Height);

    int count=0;

    while(cap.isOpened()){

        auto start = std::chrono::steady_clock::now();

        cap.read(frame);
        count++;

        if(count%2==0){
            continue;
        }

        cv::Mat img0=frame(rect0);
        cv::Mat img1=frame(rect1);

        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }

        string idx_string = PadNumber(count,6);

        cv::imwrite(save_left_path+"/"+ idx_string +".png",img0);
        cv::imwrite(save_right_path+"/"+ idx_string +".png",img1);

        //cv::imshow("Test", frame);
        //if(cv::waitKey(1)== 27) break;

        cout<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()<<endl;

    }
}


int main(int argc, char** argv) {


    VideoSplit();


    return 0;
}
