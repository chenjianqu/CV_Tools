//
// Created by cjq on 23-10-9.
//
#include <filesystem>

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

using namespace std;
namespace fs = std::filesystem;

typedef pcl::PointXYZ Point_t;
typedef pcl::PointNormal PointNormal_t;


pcl::PolygonMesh RunPoisson(const pcl::search::KdTree<PointNormal_t>::Ptr &tree,
                const pcl::PointCloud<PointNormal_t>::Ptr& cloud_with_normals){
    //创建Poisson对象，并设置参数
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8);//树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。
    // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度

    //设置搜索方法和输入点云
    //pn.setIndices();
    pn.setSearchMethod(tree);
    pn.setInputCloud(cloud_with_normals);

    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;
    //执行重构
    pn.performReconstruction(mesh);

    return mesh;
}


pcl::PolygonMesh RunFastTriangulator(const pcl::search::KdTree<PointNormal_t>::Ptr &tree2,
                            const pcl::PointCloud<PointNormal_t>::Ptr& cloud_with_normals){
     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
     // Set the maximum distance between connected points (maximum edge length)
     gp3.setSearchRadius(0.025);  // 设置连接点之间的最大距离，（即是三角形最大边长）
     // 设置各参数值
     gp3.setMu(2.5);  // 设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
     gp3.setMaximumNearestNeighbors(100);// 设置样本点可搜索的邻域个数
     gp3.setMaximumSurfaceAngle(M_PI / 4);  // 设置某点法线方向偏离样本点法线的最大角度45
     gp3.setMinimumAngle(M_PI / 18);        // 设置三角化后得到的三角形内角的最小的角度为10
     gp3.setMaximumAngle(2 * M_PI / 3);       // 设置三角化后得到的三角形内角的最大角度为120
     gp3.setNormalConsistency(false);     // 设置该参数保证法线朝向一致
     // Get result
     gp3.setInputCloud(cloud_with_normals);// 设置输入点云为有向点云
     gp3.setSearchMethod(tree2);               // 设置搜索方式

     pcl::PolygonMesh mesh;                // 存储最终三角化的网络模型
     gp3.reconstruct(mesh);               // 重建提取三角化
     // 附加顶点信息
     std::vector<int> parts = gp3.getPartIDs();
     std::vector<int> states = gp3.getPointStates();

     return mesh;
}



pcl::PolygonMesh RunMovingCube(const pcl::search::KdTree<PointNormal_t>::Ptr &tree2,
                                     const pcl::PointCloud<PointNormal_t>::Ptr& cloud_with_normals){
     pcl::MarchingCubes<pcl::PointNormal> *mc;
     mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
     /*
       if (hoppe_or_rbf == 0)
         mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
       else
       {
         mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
         (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
       }
     */
     //创建多变形网格，用于存储结果
     pcl::PolygonMesh mesh;
     //设置MarchingCubes对象的参数
     mc->setIsoLevel(0.0f);
     mc->setGridResolution(50, 50, 50);
     mc->setPercentageExtendGrid(0.0f);
     //设置搜索方法
     mc->setInputCloud(cloud_with_normals);
     //执行重构，结果保存在mesh中
     mc->reconstruct(mesh);
}


pcl::PointCloud<PointNormal_t>::Ptr GenerateNormal(const pcl::PointCloud<Point_t>::Ptr& cloud){
    pcl::NormalEstimation<Point_t, pcl::Normal> n;//法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
    pcl::search::KdTree<Point_t>::Ptr tree(new pcl::search::KdTree<Point_t>);//定义kd树指针
    tree->setInputCloud(cloud);                        //用cloud构建tree对象
    n.setInputCloud(cloud);                            //为法线估计对象设置输入点云
    n.setSearchMethod(tree);                          //设置搜索方法
    n.setKSearch(20);                                 //设置k搜索的k值为20
    n.compute(*normals);                              //估计法线存储结果到normals中

    //将点云和法线放到一起
    pcl::PointCloud<PointNormal_t>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormal_t>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
    std::cerr << "法线计算   完成" << std::endl;

    return cloud_with_normals;
}


/**
 * 点云平滑
 * @param tree
 * @param cloud_with_normals
 * @param radius
 * @return
 */
pcl::PointCloud<PointNormal_t>::Ptr SmoothPointCloud(const pcl::search::KdTree<PointNormal_t>::Ptr &tree,
                                                     const pcl::PointCloud<PointNormal_t>::Ptr& cloud_with_normals,
                                                     double radius = 0.3){
    pcl::MovingLeastSquares<PointNormal_t, PointNormal_t> mls;
    mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计
    mls.setInputCloud(cloud_with_normals);//设置参数
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);// 单位m.设置用于拟合的K近邻半径

    pcl::PointCloud<PointNormal_t>::Ptr cloud_with_normals_msl(new pcl::PointCloud<PointNormal_t>);
    mls.process(*cloud_with_normals_msl);

    return cloud_with_normals_msl;
}


int main(int argc, char** argv)
{
    fs::path input_path= "/media/cjq/新加卷/datasets/MVS/maxieye/test/sampled_cloud.ply";

    pcl::PointCloud<Point_t>::Ptr cloud(new pcl::PointCloud<Point_t>());
    pcl::io::loadPLYFile(input_path, *cloud);
    std::cerr << "Cloud before filtering: "<<cloud->size() << std::endl;

    // 计算法向量
    auto cloud_with_normals = GenerateNormal(cloud);

    //创建搜索树
    pcl::search::KdTree<PointNormal_t>::Ptr tree(new pcl::search::KdTree<PointNormal_t>);
    tree->setInputCloud(cloud_with_normals);

    //将点云进行了MLS的映射，使得输出的点云更加平滑。
    cloud_with_normals = SmoothPointCloud(tree, cloud_with_normals, 0.2);

    // 开始表面重建 ********************************************************************

    auto mesh = RunPoisson(tree, cloud_with_normals);
    pcl::io::savePLYFile(input_path.string()+"mesh.ply", mesh);//保存网格图
    std::cerr << "泊松重建完成" << std::endl;


    // // 贪婪投影三角化算法
    //auto mesh = RunFastTriangulator(tree,cloud_with_normals);
    // pcl::io::saveVTKFile(name + "-quick.ply", mesh);
    // std::cerr << "快速三角化 完成" << std::endl;


    // //移动立方体算法
    //auto mesh = RunMovingCube(tree,cloud_with_normals);
    // pcl::io::saveVTKFile(name + "-cubes.ply", mesh);
    // std::cerr << "移动立方体 完成" << std::endl;

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "my");
    viewer->addCoordinateSystem(50.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}

