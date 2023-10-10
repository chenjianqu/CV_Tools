//
// Created by cjq on 23-10-9.
//
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>

#include "def.h"

namespace o3d=open3d;
namespace otool = open3d::utility;
namespace ovis = open3d::visualization;
using o3d_Cloud_t = o3d::geometry::PointCloud;


/**
 * https://blog.csdn.net/qq_41664688/article/details/113855088
 * 调用语法Y = quantile(X,p)，其中X是向量或矩阵，p是百分位数，取值范围为[0, 1]，用于求向量或矩阵的百分位数。
该百分位数的数学意义为：对于向量X，P{X<=Y}=p
 实例：
 x = 1×6
 	2     5     6    10    11    13
y = quantile(x,0.50)
	y = 8

 为了更加直观地理解百分位数，可以给出x中每个元素对应的百分位数
    P{X<=2}=0.5/6
    P{X<=5}=1.5/6
    P{X<=6}=2.5/6=0.4167
    P{X<=10}=3.5/6=0.5833
    P{X<=11}=4.5/6
    P{X<=13}=5.5/6
而百分位数0.5对应的元素值应在6和10之间，这里直接取平均得到y=8
 * @tparam T
 * @param vec
 * @param percentage
 * @return
 *
 *
 */
template<typename T>
T quantile(const vector<T>& vec, double percentage)
{
    assert(percentage >= 0.0 && percentage <= 1.0);
    const int n = vec.size();
    double id = (n-1)*percentage;
    int lo = std::floor(id);
    int hi = std::ceil(id);
    T qs = vec[lo];
    T ps = vec[hi];
    return (1.0 - (id-lo)) * qs + (id-lo) * ps;
}



/**
 * 将数值映射到颜色
 * @tparam T 数值的类型
 * @tparam ColorT 颜色的类型
 * @param value
 * @param color_norm 是否返回归一化的颜色
 * @return
 */
template<typename T,typename ColorT>
vector<Eigen::Matrix<ColorT,3,1>> get_color_mapping(const vector<T>& value,bool color_norm = true){
    ///归一化值
    T min_value = *std::min_element(value.begin(),value.end());
    T max_value = *std::max_element(value.begin(),value.end());
    T range = max_value - min_value;
    vector<T> norm_value(value.size());
    for(int i=0;i<value.size();++i){
        norm_value[i] = (value[i] - min_value) / range;
    }

    ///将值映射到颜色
    vector<Eigen::Matrix<ColorT,3,1>> colors(norm_value.size());
    if(color_norm){
        for(int i=0;i<norm_value.size();++i) {
            colors[i].x() = 1;
            colors[i].y() = 1 - norm_value[i];
            colors[i].z() = norm_value[i];
        }
    }
    else{
        for(int i=0;i<norm_value.size();++i) {
            colors[i].x() = 255;
            colors[i].y() = int(255 - norm_value[i]*255);
            colors[i].z() = int(norm_value[i]*255);
        }
    }

    return colors;
}



int main(int argc, char *argv[])
{
    int num_thread = otool::GetProgramOptionAsInt(argc, argv, "--num_thread", 1);
    std::string path_pcd = otool::GetProgramOptionAsString(
            argc, argv, "--path_pcd",
            "/media/cjq/新加卷/datasets/MVS/maxieye/test/sampled_cloud.ply");
    if (path_pcd.empty())
        otool::LogError("path_pcd is empty, please use --path_pcd specific a correct path", path_pcd);
    otool::LogInfo("num_thread: {}, path_pcd: {}", num_thread, path_pcd);

    std::shared_ptr<o3d_Cloud_t> cloud(new o3d_Cloud_t);
    open3d::io::ReadPointCloud(path_pcd, *cloud);//读取点云

    if (cloud->IsEmpty())
        otool::LogError("can not read pointcloud from file: {}", path_pcd);
    else
        otool::LogInfo("read {} points from {}", cloud->points_.size(), path_pcd);
    TicToc tt;

    ///法线估计
    cloud->EstimateNormals();

    ///泊松重建
    /**
     *  pcd  	包含法线信息的待重建点云
        depth
        (int ，可选参数，默认=8) 曲面重建树的最大深度，在深度D处运行对应于分辨率不大于2^D*2^D*2^D的网格上进行求解。注：由于重建时根据采样密度调整OCTtree，
        因此此处指定的重建深度只是一个上限。
        width	（float ，可选参数，默认=0）指定最细级别的八叉树单元格的目标宽度；如果指定了深度，则此参数忽略。
        scale	（float，可选，默认值=1.1）指定用于重建的立方体直径与样本边界立方体直径之间的比率。
        linear_fit	（bool，可选，默认值=False）如果为true，重建时将使用线性插值来估计iso顶点的位置
        n_threads	（int，可选，默认值=-1）用于重建的线程数，设置为-1表示自动选择
     */
    auto [mesh,vertex_density] = o3d::geometry::TriangleMesh::
    CreateFromPointCloudPoisson(*cloud);

    ///mesh可视化
    mesh->PaintUniformColor({0,0,1});
    ovis::DrawGeometries({mesh}, "mesh",640,480,50,50,
                         false,true);

    ///mesh密度可视化
    auto mesh_density = std::make_shared<o3d::geometry::TriangleMesh>();
    mesh_density->vertices_ = mesh->vertices_;
    mesh_density->triangles_ = mesh->triangles_;
    mesh_density->triangle_normals_ = mesh->triangle_normals_;
    mesh_density->vertex_colors_ = get_color_mapping<double,double>(vertex_density);
    ovis::DrawGeometries({mesh_density}, "mesh_density",640,480,50,50,
                         false,true);

    ///删掉密度低的区域的顶点
    vector<bool> vertex_mask(vertex_density.size());
    auto threshold = quantile<double>(vertex_density,0.01);//计算阈值
    for(int i=0;i<vertex_density.size();++i){
        vertex_mask[i] = vertex_density[i] < threshold;
    }
    mesh->RemoveVerticesByMask(vertex_mask);
    ovis::DrawGeometries({mesh}, "mesh",640,480,50,50,
                         false,true);

    ///表面细分
    auto mesh_subdivided = mesh->SubdivideMidpoint(4);
    cout<<fmt::format("SubdivideMidpoint() {} -> {}",mesh->vertices_.size(),mesh_subdivided->vertices_.size());
    ovis::DrawGeometries({mesh_subdivided}, "mesh_subdivided",640,480,50,50,
                         false,true);

    /*auto viewer = ovis::Visualizer();
    viewer.CreateVisualizerWindow("viewer");
    auto& option = viewer.GetRenderOption();
    option.background_color_ = {0,0,0};
    option.point_size_ = 1;

    auto axis_pcd = o3d::geometry::TriangleMesh::CreateCoordinateFrame(1.,{0,0,0});
    viewer.AddGeometry(axis_pcd);
    viewer.AddGeometry(mesh);
    viewer.Run();*/

    return 0;
}
