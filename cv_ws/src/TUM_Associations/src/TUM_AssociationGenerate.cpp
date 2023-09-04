#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

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
#include <utility>

namespace fs=std::filesystem;


/**
 * 四舍五入保留n位小数
 * @param number
 * @param n_bit
 * @return
 */
double round_double(double number,int n_bit){
    static std::stringstream ss;
    ss<<std::fixed<<std::setprecision(n_bit)<<number;
    ss>>number;
    return number;
}



/**
 * 将字符串写入到文件中
 * @param path
 * @param text
 */
void WriteTextFile(const std::string& path,std::string& text){
    static std::set<std::string> path_set;
    ///第一次,清空文件
    if(path_set.find(path)==path_set.end()){
        path_set.insert(path);
        std::ofstream fout( path.data(), std::ios::out);
        fout.close();
    }

    ///添加到文件后面
    std::ofstream fout(path.data(), std::ios::app);
    fout<<text<<std::endl;
    fout.close();
}


class ImageGrabber{
public:
    ImageGrabber(fs::path save_path):save_path_(std::move(save_path)){};
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    unsigned long counter{};
    unsigned long counterKeyFrame{};

private:
    fs::path save_path_;

};



int main(int argc,char **argv)
{
    if(argc!=2){
        std::cout<<"usage:rosrun TUM_Associations TUM_AssociationGenerate ${save_path}"<<std::endl;
        return -1;
    }

    ros::init(argc, argv, "TUM_AssociationGenerate_node");

    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


    ImageGrabber igb{argv[1]};

    std::string color_topic="/camera/rgb/image_color";
    std::string depth_topic="/camera/depth/image";

    //接受RGB图和深度图
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, color_topic, 5);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, depth_topic, 5);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(30), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,igb,_1,_2));

    std::cout<<"wait image from "+color_topic+" and "+depth_topic<<std::endl;

    ros::spin();

    return 0;
}



void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    double time_rgb = round_double(msgRGB->header.stamp.toSec(),6);
    double time_depth = round_double(msgD->header.stamp.toSec(),6);

    std::string line = std::to_string(time_rgb)+" rgb/"+std::to_string(time_rgb)+".png "+
            std::to_string(time_depth)+" depth/"+std::to_string(time_depth)+".png";

    WriteTextFile(save_path_.string(),line);

    std::cout<<line<<std::endl;
}





