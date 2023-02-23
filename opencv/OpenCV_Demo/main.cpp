#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaoptflow.hpp>

#include <chrono>


using namespace std;
using namespace cv;


void flow_track_test(cv::Mat &img1,cv::Mat img2)
{
    //cv::cuda::GpuMat src,dst;
    //src.upload(img1);
    auto detector=cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1,200,0.01,20);

    vector<cv::Point_<float>> new_pts;
    detector->detect(img1,new_pts);
}


void BasicTest()
{
    Mat leftImg=imread("left.jpg",0);
    Mat rightImg=imread("right.jpg",0);

    imshow ( "leftImg", leftImg);
    imshow ( "rightImg", rightImg);
    waitKey ( 0 );

    destroyAllWindows();
}


void VideoTest()
{
    cv:: VideoCapture cap = cv::VideoCapture(1);
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cv::Mat frame, img;
    cap.read(frame);
    cout<<frame.size<<endl;

    while(cap.isOpened()){
        clock_t start = clock();
        cap.read(frame);
        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }

        cv::imshow("Test", frame);
        if(cv::waitKey(1)== 27) break;
    }
}


void VideoStereoTest()
{
    cv:: VideoCapture cap0 = cv::VideoCapture(0);
    cap0.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap0.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    cv:: VideoCapture cap1 = cv::VideoCapture(1);
    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    cv::Mat frame0, frame1;
    cap0.read(frame0);
    cout<<frame0.size<<endl;

    cap1.read(frame1);
    cout<<frame1.size<<endl;

    while(cap0.isOpened() && cap1.isOpened()){
        cap0.read(frame0);
        cap1.read(frame1);
        if(frame0.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }

        cv::imshow("Test0", frame0);
        cv::imshow("Test1", frame1);
        if(cv::waitKey(1)== 27) break;
    }
}



void VideoUVCTest()
{
    cv:: VideoCapture cap = cv::VideoCapture(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cv::Mat frame, img;
    cap.read(frame);
    cout<<frame.size<<endl;

    cv::Rect rect0(0,0,1280,720);
    cv::Rect rect1(1280,0,1280,720);

    while(cap.isOpened()){

        auto start = std::chrono::steady_clock::now();

        cap.read(frame);

        cv::Mat img0=frame(rect0);
        cv::Mat img1=frame(rect1);

        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }


        //cv::imshow("Test", frame);
        //if(cv::waitKey(1)== 27) break;

        cout<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()<<endl;
    }
}



void ReadWriteYamlTest(){
    string config_path = "/home/chen/CLionProjects/CV_Tools/OpenCV_Test/viode.yaml";
    cv::FileStorage fs(config_path,
                       cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        throw std::runtime_error(std::string("ERROR: Wrong path to settings:" + config_path));
    }

    fs<<"dataset_sequence"<<string("Test").data();


    fs.release();


}



int main ( int argc, char** argv )
{
    cout << "OpenCV version : " << CV_VERSION << endl;

    //VideoTest();

    //VideoUVCTest();

    ReadWriteYamlTest();

    return 0;
}