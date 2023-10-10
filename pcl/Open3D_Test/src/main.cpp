#include <iostream>
#include <memory>
#include <thread>
#include <open3d/Open3D.h>

using namespace std;

int main(int argc, char * argv[]) {
    std::cout << "Hello, Open3D!! " << std::endl;

    string ply_path="/home/cjq/CLionProjects/CV_Tools/pcl/Open3D_Test/data/filtered_cloud.ply";

    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
    auto pcd = open3d::io::CreatePointCloudFromFile(ply_path);

    // 1. test downsample
    auto downsampled = pcd->VoxelDownSample(0.05);

    ///特征提取
    {
        open3d::utility::ScopeTimer timer("FPFH estimation with Radius 0.25");
//        open3d::pipelines::registration::ComputeFPFHFeature(
//                *downsampled,
//                open3d::geometry::KDTreeSearchParamRadius(0.25));
    }

    // 2. 估计点云的法向量
    {
        open3d::utility::ScopeTimer timer("Normal estimation with KNN20");
        for (int i = 0; i < 20; i++){
            downsampled->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));
        }
    }
    std::cout << downsampled->normals_[0] << std::endl;
    std::cout << downsampled->normals_[10] << std::endl;

    {
        open3d::utility::ScopeTimer timer("Normal estimation with Radius 0.01666");
        for(int i=0; i<20; i++){
            downsampled->EstimateNormals(open3d::geometry::KDTreeSearchParamRadius(0.01666));
        }
    }
    std::cout << downsampled->normals_[0] << std::endl;
    std::cout << downsampled->normals_[10] << std::endl;

    open3d::visualization::DrawGeometries({downsampled}, "TestPCD", 1920, 1080);

    return 0;
}
