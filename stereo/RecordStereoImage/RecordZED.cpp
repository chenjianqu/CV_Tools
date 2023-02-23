#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace std;
using namespace cv;



int RecordZED(int argc, char** argv)
{



    string left_save_path;
    string right_save_path ;

    if(argc ==3){
        left_save_path = argv[1];
        right_save_path = argv[2];
    }
    else{
        cerr<<"please input the save_path"<<endl;
    }

    cv:: VideoCapture cap = cv::VideoCapture(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cv::Mat frame, img;
    cap.read(frame);
    cout<<frame.size<<endl;

    cv::Rect rect0(0,0,1280,720);
    cv::Rect rect1(1280,0,1280,720);

    int count=0;

    while(cap.isOpened()){
        auto start = std::chrono::steady_clock::now();

        cap.read(frame);

        cv::Mat img0=frame(rect0);
        cv::Mat img1=frame(rect1);

        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }

        if(argc==3){
            cv::imwrite(left_save_path+"/"+ to_string(count)+".png",img0);
            cv::imwrite(right_save_path+"/"+ to_string(count)+".png",img1);
        }
        else{
            cv::imshow("Test", frame);
            if(cv::waitKey(1)== 27) break;
        }

        cout<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()<<endl;

        count++;
    }
}


int main(int argc, char** argv) {

    return VideoUVCTest(argc,argv);
}
