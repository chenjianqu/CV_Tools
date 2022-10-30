#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include "camera_model.h"
#include "io_utils.h"

using namespace std;

using PointT = pcl::PointXYZRGB;
using PointCloud=pcl::PointCloud<PointT>;


PointCloud::Ptr StereoCalculatePointCloud(cv::Mat &left,cv::Mat &disp){
    PointCloud::Ptr cloud (new PointCloud);

    for (int v = 0; v < left.rows; v++){
        for (int u = 0; u < left.cols; u++) {
            if (disp.at<float>(v, u) <= 0.0 || disp.at<float>(v, u) >= 96.0) continue;
            // 根据双目模型计算 point 的位置
            float x = (u - cam0->cx) / cam0->fx;
            float y = (v - cam0->cy) /cam0->fy;
            float depth = cam0->fx * cam1->baseline / (disp.at<float>(v, u));
            auto pixel = left.at<cv::Vec3b>(v,u);
            PointT p(pixel[0],pixel[1],pixel[2]);
            p.x = x * depth;
            p.y = y * depth;
            p.z = depth;
            cloud->push_back(p);
            if(v==u){
                cout<<depth<<endl;
            }
        }
    }

    return cloud;
}

int main(int argc,char** argv) {
    std::cout << "Hello, World!" << std::endl;

    if(argc!=4){
        cerr<<"parameters number should be 4"<<endl;
        return -1;
    }

    string left_image_path=argv[1];
    string stereo_image_path=argv[2];
    string calib_file_path=argv[3];

    InitCamera(calib_file_path);

    vector<fs::path> left_names=GetDirectoryFileNames(left_image_path);

    pcl::visualization::CloudViewer viewer ("test");


    for(auto &name:left_names){
        cv::Mat left = cv::imread(left_image_path+name.string());
        cv::Mat disp_raw = cv::imread(stereo_image_path+name.string(),-1);
        cv::Mat disp;
        disp_raw.convertTo(disp, CV_32F,1./256.);


        auto cloud=StereoCalculatePointCloud(left,disp);
        cout<<cloud->points.size()<<endl;

        viewer.showCloud(cloud);
        while (!viewer.wasStopped()){ };
    }






    return 0;
}
