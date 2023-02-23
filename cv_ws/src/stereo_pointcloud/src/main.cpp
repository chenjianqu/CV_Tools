#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>

#include "io_utils.h"

#include "def.h"

using namespace std;
namespace fs=std::filesystem;

using PointT = pcl::PointXYZRGB;
using PointCloud=pcl::PointCloud<PointT>;

ros::NodeHandle *nh;

std::unordered_map<string,ros::Publisher> pub_map;

//相机内参
float cx,cy,fx,fy;
float baseline=0.08f;
float k1,k2,p1,p2;

bool ReadFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()){
        return false;
    }
    cv::FileNode n = fs["distortion_parameters"];
    k1 = static_cast<float>(n["k1"]);
    k2 = static_cast<float>(n["k2"]);
    p1 = static_cast<float>(n["p1"]);
    p2 = static_cast<float>(n["p2"]);

    n = fs["projection_parameters"];
    fx = static_cast<float>(n["fx"]);
    fy = static_cast<float>(n["fy"]);
    cx = static_cast<float>(n["cx"]);
    cy = static_cast<float>(n["cy"]);

    return true;
}


void Pub(PointCloud &cloud,const string &topic){
    if(pub_map.find(topic)==pub_map.end()){
        ros::Publisher pub = nh->advertise<sensor_msgs::PointCloud2>(topic,10);
        pub_map.insert({topic,pub});
    }

    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(cloud,point_cloud_msg);
    point_cloud_msg.header = header;

    pub_map[topic].publish(point_cloud_msg);
}


PointCloud::Ptr StereoCalculatePointCloud(cv::Mat &left,cv::Mat &disp){
    PointCloud::Ptr cloud (new PointCloud);

    for (int v = 0; v < left.rows; v++){
        for (int u = 0; u < left.cols; u++) {
            if (disp.at<float>(v, u) <= 0.5 || disp.at<float>(v, u) >= 96.0)
                continue;
            // 根据双目模型计算 point 的位置
            float x = (u - cx) / fx;
            float y = (v - cy) /fy;
            float depth = fx * baseline / (disp.at<float>(v, u));
            auto pixel = left.at<cv::Vec3b>(v,u);
            PointT p(pixel[0],pixel[1],pixel[2]);
            p.x = x * depth;
            p.y = y * depth;
            p.z = depth;
            cloud->push_back(p);
            //if(v==u){
            //    cout<<depth<<endl;
            //}
        }
    }

    return cloud;
}



int main(int argc, char **argv)
{
    if(argc!=4){
        cerr<<"parameters number should be 4"<<endl;
        cout<<"usage:rosrun stereo_pointcloud stereo_to_pointcloud ${left_image_dir} ${stereo_image_path} ${calib_file_path}"<<endl;
        return -1;
    }

    ros::init(argc, argv, "stereo_pointcloud_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    nh = &n;

    string left_image_dir=argv[1];
    string disp_image_dir=argv[2];
    string calib_file_path=argv[3];

    ReadFromYamlFile(calib_file_path);

    vector<fs::path> left_names=GetDirectoryFileNames(left_image_dir);

    ros::Rate rate(30);
    for(auto &name:left_names){
        if(!ros::ok()){
            break;
        }

        fs::path left_path = left_image_dir / name;
        if(!fs::exists(left_path)){
            cerr<<left_path<<" is not exists"<<endl;
            break;
        }
        fs::path disp_path = disp_image_dir / name;
        if(!fs::exists(disp_path)){
            cerr<<disp_path<<" is not exists"<<endl;
            break;
        }

        cout<<disp_path<<endl;

        cv::Mat left = cv::imread(left_path.string());
        cv::Mat disp_raw = cv::imread(disp_path.string(), -1);
        cv::Mat disp;
        disp_raw.convertTo(disp, CV_32F,1./256.);

        auto cloud=StereoCalculatePointCloud(left,disp);
        cout<<cloud->points.size()<<endl;

        Pub(*cloud,"/stereo_pointcloud");

        ros::spinOnce();
        rate.sleep();

    }

}




