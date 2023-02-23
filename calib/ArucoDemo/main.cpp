#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;

int main() {

    cv::Mat img = cv::imread("/home/chen/图片/2022-12-02 19-28-12屏幕截图.png");

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> marker_corners,rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);//字典对象，
    cv::aruco::detectMarkers(img,
                             dictionary,
                             marker_corners,//检测出的图像的角点的列表
                             markerIds//检测出来的marker的列表
                            );

    for(int i=0;i<markerIds.size();++i){
        int id = markerIds[i];
        cout<<"id:"<<id<<endl;
        std::vector<cv::Point2f> corner = marker_corners[i];
        cv::line(img,corner[0],corner[1],cv::Scalar(255,0,0),2);
        cv::line(img,corner[1],corner[2],cv::Scalar(255,0,0),2);
        cv::line(img,corner[2],corner[3],cv::Scalar(255,0,0),2);
        cv::line(img,corner[3],corner[0],cv::Scalar(255,0,0),2);
    }

    cv::imshow("test",img);
    cv::waitKey(0);


    std::cout << "markerIds.size():"<<markerIds.size() << std::endl;
    return 0;
}
