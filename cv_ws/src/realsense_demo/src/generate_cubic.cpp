#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


using namespace std;


class ImageGrabber{
public:
    ImageGrabber();
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    unsigned long counter;
    unsigned long counterKeyFrame;
protected:
    image_transport::Publisher* pub_rgb;
    image_transport::Publisher* pub_depth;
    ros::Publisher *pub_pc;

    double costSumTime;

};



void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr= cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img=ptr->image.clone();

    /*cv::cuda::GpuMat d_img(img);
    cv::cuda::bitwise_not(d_img,d_img);
    d_img.download(img);*/

    double time = img_msg->header.stamp.toSec();
    std::ostringstream oss;
    oss << std::fixed <<std::setprecision(6) << time;
    string time_str1 = oss.str();

    //string time_str = fmt::format("{:.6f}",time);

    string time_str = std::to_string(time);

    cout<<"time_str1:"<<time_str1<<endl;
    cout<<"time_str:"<<time_str<<endl;


    cv::imshow("image",img);
    cv::waitKey(1);
}



int main(int argc,char **argv)
{
    /*cv::Mat img(100,100,CV_8UC1);
    cv::cuda::GpuMat d_img(img);
    cv::cuda::bitwise_not(d_img,d_img);
    d_img.download(img);*/

    ros::init(argc, argv, "genera_cubic");

    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ImageGrabber igb;

    //接受RGB图和深度图
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/camera/depth/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    cout<<"wait image"<<endl;

    ros::spin();

    return 0;
}




ImageGrabber::ImageGrabber():
counter(0),
counterKeyFrame(0),
costSumTime(0.0)
{
}



void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    ros::Time timestamp= msgRGB->header.stamp;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try{
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try{
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    time_t beginTime=clock();

    //调用ORB-SLAM2
    //cv::Mat Tcw=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    cv::Mat img_color = cv_ptrRGB->image;
    cv::Mat img_depth = cv_ptrD->image;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> marker_corners,rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);//字典对象，
    cv::aruco::detectMarkers(img_color,
                             dictionary,
                             marker_corners,//检测出的图像的角点的列表
                             markerIds  //检测出来的marker的列表
                             );

    for(int i=0;i<markerIds.size();++i){
        int id = markerIds[i];
        cout<<"id:"<<id<<endl;
        std::vector<cv::Point2f> corner = marker_corners[i];
        cv::line(img_color,corner[0],corner[1],cv::Scalar(255,0,0),2);
        cv::line(img_color,corner[1],corner[2],cv::Scalar(255,0,0),2);
        cv::line(img_color,corner[2],corner[3],cv::Scalar(255,0,0),2);
        cv::line(img_color,corner[3],corner[0],cv::Scalar(255,0,0),2);
    }

    cout<<markerIds.size()<<endl;

    cv::imshow("test",img_color);
    cv::waitKey(2);





    counterKeyFrame++;
    counter++;

}





