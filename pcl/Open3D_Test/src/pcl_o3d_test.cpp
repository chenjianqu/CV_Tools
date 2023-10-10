//
// Created by cjq on 23-10-9.
//
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <open3d/Open3D.h>

#include <pcl/point_types.h>          //PCL中所有点类型定义的头文件
#include <pcl/io/pcd_io.h>            //打开关闭pcd文件的类定义的头文件
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>//视觉化工具函式库（VTK，Visualization Toolkit）　模型
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/search/kdtree.h>//kdtree搜索对象的类定义的头文件
#include <pcl/features/normal_3d.h>//法向量特征估计相关类定义的头文件
//重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>          //最小二乘法平滑处理类定义头文件
#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>

#include "def.h"

namespace o3d=open3d;
namespace otool = open3d::utility;
namespace ovis = open3d::visualization;
using o3d_Cloud_t = o3d::geometry::PointCloud;


namespace pcd_converter
{
    void ThreadAssignment(int num_thread, int data_size, std::vector<int> &data_in_each_thread)
    {
        // 为每个线程分配的数据量 std::vector<int> data_N;
        // 如果数据量小于线程数,每个线程只用处理一个数据
        if (data_size <= num_thread){
            data_in_each_thread = std::vector<int>(data_size, 1);
            return;
        }
        data_in_each_thread.clear();
        if (0 == data_size % num_thread){
            data_in_each_thread = std::vector<int>(num_thread, data_size / num_thread);
        }
        else{
            data_in_each_thread = std::vector<int>(num_thread, data_size / num_thread + 1
                - (num_thread > 10 ? 1 : 0));
            data_in_each_thread[num_thread - 1] = data_size - (data_size / num_thread + 1 -
                    (num_thread > 10 ? 1 : 0)) * (num_thread - 1);
        }
    }


    /// @brief 将open3d点云转为pcl点云
    /// @param pc
    /// @param num_thread 使用的线程数量
    /// @return
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    ConvertO3dToPclParallel(std::shared_ptr<o3d_Cloud_t> pc, int num_thread = 5){
        // 给线程分配数据
        std::vector<int> data_in_each_thread;
        ThreadAssignment(num_thread, pc->points_.size(), data_in_each_thread);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_ptr->resize(pc->points_.size());

        auto convert_thread = [&](int id, std::vector<int> data){
            for (std::size_t i = id * data[0]; i < id * data[0] + data[id]; ++i){
                cloud_ptr->points[i].x = pc->points_[i].x();
                cloud_ptr->points[i].y = pc->points_[i].y();
                cloud_ptr->points[i].z = pc->points_[i].z();
            }
        };

        std::vector<std::thread> vec_convert_thread;
        for (std::size_t i = 0; i < data_in_each_thread.size(); ++i)
            vec_convert_thread.emplace_back(convert_thread, i, data_in_each_thread);
        for (auto & i : vec_convert_thread)
            i.join();
        return cloud_ptr;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr
    ConvertO3dToPcl(const std::shared_ptr<o3d_Cloud_t>& pc){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_ptr->resize(pc->points_.size());

        for(int i=0;i<pc->points_.size();++i){
            cloud_ptr->points[i].x = pc->points_[i].x();
            cloud_ptr->points[i].y = pc->points_[i].y();
            cloud_ptr->points[i].z = pc->points_[i].z();
        }
        return cloud_ptr;
    }


    /// @brief 将pcl点云转为open3d
    /// @param pc
    /// @param num_thread 使用的线程数量
    /// @return
    std::shared_ptr<o3d_Cloud_t> ConvertPclToO3dParallel(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int num_thread = 5)
    {
        // 给线程分配数据
        std::vector<int> data_in_each_thread;
        ThreadAssignment(num_thread, pc->points.size(), data_in_each_thread);

        auto open3d_cloud_ptr = std::make_shared<o3d_Cloud_t>();
        open3d_cloud_ptr->points_.resize(pc->points.size());

        auto convert_thread = [&](int id, std::vector<int> data){
            for (std::size_t i = id * data[0]; i < id * data[0] + data[id]; ++i){
                open3d_cloud_ptr->points_[i][0] = pc->points[i].x;
                open3d_cloud_ptr->points_[i][1] = pc->points[i].y;
                open3d_cloud_ptr->points_[i][2] = pc->points[i].z;
            }
        };

        std::vector<std::thread> vec_convert_thread;
        for (std::size_t i = 0; i < data_in_each_thread.size(); ++i)
            vec_convert_thread.emplace_back(convert_thread, i, data_in_each_thread);

        for (auto & i : vec_convert_thread)
            i.join();
        return open3d_cloud_ptr;
    }


    std::shared_ptr<o3d_Cloud_t> ConvertPclToO3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
    {
        auto open3d_cloud_ptr = std::make_shared<o3d_Cloud_t>();
        open3d_cloud_ptr->points_.resize(pc->points.size());
        for (std::size_t i = i; i < pc->size(); ++i){
            open3d_cloud_ptr->points_[i][0] = pc->points[i].x;
            open3d_cloud_ptr->points_[i][1] = pc->points[i].y;
            open3d_cloud_ptr->points_[i][2] = pc->points[i].z;
        }
        return open3d_cloud_ptr;
    }

} // namespace pcd_converter



int main(int argc, char *argv[])
{
    int num_thread = otool::GetProgramOptionAsInt(argc, argv, "--num_thread", 1);
    std::string path_pcd = otool::GetProgramOptionAsString(
            argc, argv, "--path_pcd",
            "/media/cjq/新加卷/datasets/MVS/maxieye/test/sampled_cloud.ply");
    if (path_pcd.empty())
        otool::LogError("path_pcd is empty, please use --path_pcd specific a correct path", path_pcd);
    otool::LogInfo("num_thread: {}, path_pcd: {}", num_thread, path_pcd);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    //读取点云
    std::shared_ptr<o3d_Cloud_t> pcd_o3d(new o3d_Cloud_t);
    open3d::io::ReadPointCloud(path_pcd, *pcd_o3d);

    if (pcd_o3d->IsEmpty())
        otool::LogError("can not read pointcloud from file: {}", path_pcd);
    else
        otool::LogInfo("read {} points from {}", pcd_o3d->points_.size(), path_pcd);
    TicToc tt;

    for (int i = 0; i < 5; ++i){
        tt.Tic();

        pcd_pcl = pcd_converter::ConvertO3dToPclParallel(pcd_o3d, num_thread);
        otool::LogInfo("ConvertO3dToPclParallel {} , use time:{:.2f} ms", pcd_o3d->points_.size(),tt.TocThenTic());

        pcd_pcl = pcd_converter::ConvertO3dToPcl(pcd_o3d);
        otool::LogInfo("ConvertO3dToPcl {} , use time:{:.2f} ms", pcd_o3d->points_.size(),tt.TocThenTic());

        pcd_o3d = pcd_converter::ConvertPclToO3dParallel(pcd_pcl, num_thread);
        otool::LogInfo("ConvertPclToO3dParallel {} , use time:{:.2f} ms", pcd_o3d->points_.size(),tt.TocThenTic());

        pcd_o3d = pcd_converter::ConvertPclToO3d(pcd_pcl);
        otool::LogInfo("ConvertPclToO3d {} , use time:{:.2f} ms", pcd_o3d->points_.size(),tt.TocThenTic());

        ovis::DrawGeometries({pcd_o3d}, "form pcl");

        cout<<endl;
    }

    return 0;
}
