//
// Created by chen on 2021/10/26.
//

#include <cstdio>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>


using namespace std;

using PointCloud=pcl::PointCloud<pcl::PointXYZRGB>;


const float baseline=0.05f;

class TicToc{
public:
    TicToc(){
        tic();
    }
    void tic(){
        start = std::chrono::system_clock::now();
    }
    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
    void toc_print_tic(const char* str){
        printf("%s : %f ms\n",str,toc());
        tic();
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};




class Camera{
public:
    using Ptr=std::shared_ptr<Camera>;
    Camera(float fx_,float fy_,float cx_,float cy_):fx(fx_),fy(fy_),cx(cx_),cy(cy_)
    {
    }
    void pixel2Cam(float u,float v,float disp_value,cv::Point3f &point) const{
        float x=(u-cx)/fx;
        float y=(v-cy)/fy;
        point.z=fx * baseline / disp_value;
        point.x=x*point.z;
        point.y=y*point.z;
    }

    float fx,fy,cx,cy;
    float k1,k2,p1,p2;
    int width,heigh;
};

struct IMU_GPS_Data{
  float lat;//:     latitude of the oxts-unit (deg)
  float lon;//:     longitude of the oxts-unit (deg)
  float alt;//:     altitude of the oxts-unit (m)
  float roll;//:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  float pitch;//:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  float yaw;//:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  float vn;//:      velocity towards north (m/s)
  float ve;//:      velocity towards east (m/s)
  float vf;//:      forward velocity, i.e. parallel to earth-surface (m/s)
  float vl;//:      leftward velocity, i.e. parallel to earth-surface (m/s)
  float vu;//:      upward velocity, i.e. perpendicular to earth-surface (m/s)
  float ax;//:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
  float ay;//:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
  float az;//:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
  float af;//:      forward acceleration (m/s^2)
  float al;//:      leftward acceleration (m/s^2)
  float au;//:      upward acceleration (m/s^2)
  float wx;//:      angular rate around x (rad/s)
  float wy;//:      angular rate around y (rad/s)
  float wz;//:      angular rate around z (rad/s)
  float wf;//:      angular rate around forward axis (rad/s)
  float wl;//:      angular rate around leftward axis (rad/s)
  float wu;//:      angular rate around upward axis (rad/s)
  float posacc;//:  velocity accuracy (north/east in m)
  float velacc;//:  velocity accuracy (north/east in m/s)
  int navstat;//: navigation status
  int numsats;//: number of satellites tracked by primary GPS receiver
  int posmode;//: position mode of primary GPS receiver
  int velmode;//: velocity mode of primary GPS receiver
  int orimode;//: orientation mode of primary GPS receiver
};






void ReadPointCloudBin(const std::string &path,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    ifstream input(path,std::ios::binary);
    if(!input.is_open()){
        cerr<<"Open:"<<path<<" failure"<<endl;
    }

    while(input.good() && !input.eof())
    {
        pcl::PointXYZI point;
        input.read((char*)&point.x,3*sizeof(float));
        input.read((char*)&point.intensity,sizeof(float));
        cloud->push_back(point);
    }
    input.close();
}


void WritePointCloudBin (const std::string &path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    ofstream bin_file(path.c_str(),ios::out|ios::binary|ios::app);
    if(!bin_file.good())
        cout<<"Couldn't open "<<path<<endl;

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        bin_file.write((char*)&cloud->points[i].x,3*sizeof(float));
        bin_file.write((char*)&cloud->points[i].intensity,sizeof(float));
    }
    bin_file.close();
}




void ReadTimestamps(const std::string &path,std::vector<double> &time_vector)
{
    ifstream file(path);
    if(!file.is_open()){
        printf("Can not read %s\n",path.c_str());
        return;
    }
    while(!file.eof()){
        string a,b;
        file>>a>>b;
        if(b.empty())
            continue;
        double time= stod(b.substr(b.size() - 12));
        time_vector.push_back(time);
        //cout<<time<<endl;
    }
    cout<<time_vector.size()<<endl;
    file.close();
}

IMU_GPS_Data&& ReadImuGPS(const std::string &path)
{
    ifstream file(path);
    if(!file.is_open()){
        printf("Can not read %s\n",path.c_str());
        return IMU_GPS_Data();
    }
    else{
        IMU_GPS_Data data;
        file>>data.lat >>data.lon >>data.alt >>data.roll >>data.pitch >>data.yaw
        >> data.vn>>data.ve >>data.vf >>data.vl >>data.vu >>data.ax >>data.ay >>data.az  >>data.af >>data.al >>data.au
        >> data.wx >>data.wy>>data.wz >>data.wf >>data.wl >>data.wu >>data.posacc >>data.velacc
        >>data.navstat >>data.numsats >>data.posmode >>data.velmode >>data.orimode;
        file.close();
        return std::move(data);
    }
}


void PubViewLineMarker(ros::Publisher &pub)
{
    visualization_msgs::Marker msg;
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id="world";
    msg.ns="box_strip";
    msg.action=visualization_msgs::Marker::ADD;
    msg.pose.orientation.w=1.0;

    msg.id=6666;
    msg.lifetime=ros::Duration(20);//持续时间3s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::LINE_STRIP;//marker的类型
    msg.scale.x=0.1;//线宽
    msg.color.r=1;msg.color.g=1;msg.color.b=1;//颜色:0-1
    msg.color.a=1.0;//不透明度

    //设置立方体的八个顶点
    geometry_msgs::Point p[3];
    p[0].x=10;
    p[0].y=-10;
    p[0].z=0;
    p[1].x=0;
    p[1].y=0;
    p[1].z=0;
    p[2].x=10;
    p[2].y=10;
    p[2].z=0;

    for(auto &pt : p)
        msg.points.push_back(pt);

    pub.publish(msg);
}


void PubCarModel(ros::Publisher &pub)
{
    visualization_msgs::Marker msg;
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id="world";
    msg.ns="mesh";
    msg.action=visualization_msgs::Marker::ADD;
    msg.pose.orientation.w=1.0;

    msg.id=7777;
    msg.lifetime=ros::Duration(20);//持续时间3s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::MESH_RESOURCE;
    msg.mesh_resource="package://kitti_test/mesh/Car.dae";
    //因为自车的velodyne激光雷达相对于map的位置是（0，0，0），看设备安装图上velodyne的位置是（0，0，1.73），显而易见，在rviz中自车模型位置应该是（0，0，-1.73）
    msg.pose.position.x=0.;
    msg.pose.position.y=0.;
    msg.pose.position.z=-1.73;

    auto q=tf::createQuaternionFromRPY(M_PI,M_PI,-M_PI/2);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    msg.color.r=1.;
    msg.color.g=1.;
    msg.color.b=1.;
    msg.color.a=1.;
    //设置车辆大小
    msg.scale.x=1.;
    msg.scale.y=1.;
    msg.scale.z=1.;

    pub.publish(msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_test");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


    ros::Publisher pub_point_cloud=n.advertise<sensor_msgs::PointCloud2>("stereo_point_cloud", 100);

    std::string path="/media/chen/EC4A17F64A17BBF0/datasets/kitti/raw/2011_09_26/2011_09_26_drive_0001_sync";
    std::string img0_dir_path=path+"/image_00/data/";
    std::string img1_dir_path=path+"/image_01/data/";
    std::string point_cloud_dir_path=path+"/velodyne_points/data/";
    std::string oxts_dir_path=path+"/oxts/data/";

    ///
    std::string img0_timestamp_path=path+"/image_00/timestamps.txt";
    std::string img1_timestamp_path=path+"/image_01/timestamps.txt";
    vector<double> timestamp0,timestamp1;
    ReadTimestamps(img0_timestamp_path,timestamp0);
    ReadTimestamps(img1_timestamp_path,timestamp1);

    auto pub_image0 = n.advertise<sensor_msgs::Image>("kitti/cam0", 1000);
    auto pub_image1 = n.advertise<sensor_msgs::Image>("kitti/cam1", 1000);

    auto pub_IMU=n.advertise<sensor_msgs::Imu>("kitti/IMU",1000);
    auto pub_GPS=n.advertise<sensor_msgs::NavSatFix>("kitti/GPS",1000);
    auto pub_Viewline=n.advertise<visualization_msgs::Marker>("ktti/ViewLine",1000);
    auto pub_CarModel=n.advertise<visualization_msgs::Marker>("ktti/CarModel",1000);


    int index=0;
    ros::Rate rate(10);
    while(ros::ok())
    {
        cout<<index<<endl;
        std_msgs::Header header;
        header.frame_id = "world";
        //header.stamp = ros::Time(t);
        header.stamp=ros::Time::now();

        if(index>=timestamp0.size() || index>=timestamp1.size())
            break;
        char name[64];
        sprintf(name,"%010d.png",index);
        std::string img0_path=img0_dir_path+name;
        cv::Mat img0=cv::imread(img0_path);
        sprintf(name,"%010d.png",index);//0表示填充0,10表示输出长度为10
        std::string img1_path=img1_dir_path+name;
        cv::Mat img1=cv::imread(img1_path);

        if(img0.empty()){
            cout<<"Read:"<<img0_path<<" failure"<<endl;
            break;
        }
        if(img1.empty()){
            cout<<"Read:"<<img1_path<<" failure"<<endl;
            break;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        sprintf(name,"%010d.bin",index);
        std::string pc_path=point_cloud_dir_path+name;
        ReadPointCloudBin(pc_path,cloud);

        sprintf(name,"%010d.txt",index);
        std::string oxts_path=oxts_dir_path+name;
        IMU_GPS_Data imuGpsData= ReadImuGPS(oxts_path);

        sensor_msgs::Imu imu_data;
        imu_data.header=header;
        //设置IMU数据的姿态
        auto q=tf::createQuaternionFromRPY(imuGpsData.roll,imuGpsData.pitch,imuGpsData.yaw);
        imu_data.orientation.x =q.x();
        imu_data.orientation.y =q.y();
        imu_data.orientation.z =q.z();
        imu_data.orientation.w =q.w();
        imu_data.linear_acceleration.x = imuGpsData.af;
        imu_data.linear_acceleration.y = imuGpsData.al;
        imu_data.linear_acceleration.z = imuGpsData.au;
        imu_data.angular_velocity.x = imuGpsData.wf;
        imu_data.angular_velocity.y = imuGpsData.wl;
        imu_data.angular_velocity.z = imuGpsData.wu;
        pub_IMU.publish(imu_data);

        sensor_msgs::NavSatFix gps_data;
        gps_data.header=header;
        gps_data.latitude = imuGpsData.lat;
        gps_data.longitude=imuGpsData.lon;
        gps_data.altitude=imuGpsData.alt;
        pub_GPS.publish(gps_data);

        sensor_msgs::ImagePtr img0_msg = cv_bridge::CvImage(header, "bgr8", img0).toImageMsg();
        pub_image0.publish(img0_msg);
        sensor_msgs::ImagePtr img1_msg = cv_bridge::CvImage(header, "bgr8", img1).toImageMsg();
        pub_image1.publish(img1_msg);

        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*cloud,point_cloud_msg);
        point_cloud_msg.header = header;
        pub_point_cloud.publish(point_cloud_msg);

        PubViewLineMarker(pub_Viewline);
        PubCarModel(pub_CarModel);

        ros::spinOnce();
        rate.sleep();

        index++;

    }



    return 0;
}




