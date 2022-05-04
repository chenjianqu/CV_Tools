#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaoptflow.hpp>



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
    cv:: VideoCapture cap = cv::VideoCapture(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cv::Mat frame, img;
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




int main ( int argc, char** argv )
{
    cout << "OpenCV version : " << CV_VERSION << endl;

    VideoTest();

    return 0;
}