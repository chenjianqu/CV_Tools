//
// Created by chen on 2021/10/19.
//


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

cv::VideoWriter writer;
int cnt=0;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cout<<cnt++<<endl;
    cv_bridge::CvImageConstPtr ptr= cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img=ptr->image.clone();
    writer<<img;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "multi_image_node");

    if(argc!=6){
        cout<<"参数:话题名 视频文件名 宽 高 帧率"<<endl;
        std::terminate();
    }

	int w=atoi(argv[3]);
	int h=atoi(argv[4]);
	int rate=atoi(argv[5]);


    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    writer.open(argv[2], CV_FOURCC('M','P','4','2'),rate,cv::Size(w,h));
//writer.open("/home/chen/result.mkv", CV_FOURCC('X','2','6','4'),20,cv::Size(1504,480));


    ros::Subscriber sub_img0 = n.subscribe(argv[1], 100, img0_callback);



    ros::spin();

    writer.release();

    return 0;


}







