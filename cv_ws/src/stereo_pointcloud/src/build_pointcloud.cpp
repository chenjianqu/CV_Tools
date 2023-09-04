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


PointCloud::Ptr StereoCalculatePointCloud(cv::Mat &left,cv::Mat &disp){
    PointCloud::Ptr cloud (new PointCloud);

    for (int v = 0; v < 1242; v++){
        for (int u = 0; u < 375; u++) {
            float disparity = disp.at<float>(v, u);
            if(disparity!=disparity || disparity<0.01) //值为NaN:disparity!=disparity
                continue;
            float depth = fx * baseline / disparity;
            if (depth <= 0.5 || depth >= 96.0)
                continue;
            // 根据双目模型计算 point 的位置
            float x = (u - cx) / fx;
            float y = (v - cy) /fy;

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
    ros::init(argc, argv, "build_pointcloud_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    nh = &n;

    cout<<"argc:"<<argc<<endl;
    if(argc!=5){
        for(int i=0;i<argc;++i){
            cerr<<argv[i]<<" ";
        }
        cerr<<" 。parameters number should be 5"<<endl;
        cout<<"usage:rosrun stereo_pointcloud build_pointcloud ${left_image_path} ${stereo_image_path} ${calib_file_path} ${baseline}"<<endl;
        return -1;
    }

    fs::path left_image_path=argv[1];
    fs::path disp_image_path=argv[2];
    fs::path calib_file_path=argv[3];
    baseline = stof(argv[4]);

    ReadFromYamlFile(calib_file_path);


    if(!fs::exists(left_image_path)){
        cerr<<left_image_path<<" is not exists"<<endl;
        return -1;
    }
    if(!fs::exists(disp_image_path)){
        cerr<<disp_image_path<<" is not exists"<<endl;
        return -1;
    }

    cout<<disp_image_path<<endl;

    cv::Mat left = cv::imread(left_image_path.string());

    cv::Mat disp_raw = cv::imread(disp_image_path.string(), -1);
    cv::Mat disp;
    disp_raw.convertTo(disp, CV_32F,1./16.);

    auto cloud=StereoCalculatePointCloud(left,disp);
    cout<<cloud->points.size()<<endl;

    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };

}




