#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;

using PointT = pcl::PointXYZRGB;

int main(int argc, char **argv)
{
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;// 内参
    double b = 0.573;// 基线

    // 读取图像
    string left_file = "/home/chen/datasets/kitti/stereo/data_stereo_flow/testing/colored_0/000009_10.png";
    string disp_file = "/home/chen/datasets/kitti/stereo/data_stereo_flow/testing/psmnet_output/000009_10.png";

    cv::Mat left = cv::imread(left_file);
    cv::Mat disp_raw = cv::imread(disp_file,-1);
    cv::Mat disp;
    disp_raw.convertTo(disp, CV_32F,1./256.);

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    for (int v = 0; v < left.rows; v++){
        for (int u = 0; u < left.cols; u++) {
            if (disp.at<float>(v, u) <= 0.0 || disp.at<float>(v, u) >= 96.0) continue;
            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disp.at<float>(v, u));
            auto pixel = left.at<cv::Vec3b>(v,u);
            PointT p(pixel[0],pixel[1],pixel[2]);
            p.x = x * depth;
            p.y = y * depth;
            p.z = depth;
            cloud->push_back(p);
        }
    }

    cout<<cloud->points.size()<<endl;

    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };

    return 0;
}
