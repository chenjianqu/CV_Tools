#include <iostream>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>



/**
 * 这里构建了一个交互式ICP过程，即可视化ICP的过程。
 */


bool next_iteration = false;

// 此函数是可视化窗口的回调函数。当可视化窗口被激活时，只要按空格键，就会调用此函数，将布尔变量next_iteration 设置为 true
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}


void remove_nan(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2){

    // 如果想正确使用 pcl::removeNaNFromPointCloud 函数，最好先把点云的 is_dense 属性设置为 false
    // 因为 pcl::removeNaNFromPointCloud 函数会先检查 is_dense 属性，如果该属性原本为 true 那么该函数将不会对点云进行任何处理
    cloud_1->is_dense = false;
    cloud_2->is_dense = false;

    std::vector<int> indices_1;
    pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1, indices_1);

    std::vector<int> indices_2;
    pcl::removeNaNFromPointCloud(*cloud_2, *cloud_2, indices_2);

    cloud_1->is_dense = true;
    cloud_2->is_dense = true;
}


void icp_function(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_source,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_target){
    int iterations = 10;

    // ICP 算法
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations (iterations);  // 设置要执行的初始迭代次数（默认值为1）
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.align(*cloud_source);
    icp.setMaximumIterations(1);  // 在第一次对齐之后，将这个变量设置为1，下次调用.align（）函数的时候就只会迭代一次

    if (icp.hasConverged ()){
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_source -> cloud_target" << std::endl;

        Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation ().cast<double>();
        std::cout << transformation_matrix << std::endl;
    }
    else{
        std::cerr << "\nICP has not converged." << std::endl;
    }

    // 可视化部分
    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white(cloud_target, 250, 250, 250);// 目标点云为白色
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_target, white, "cloud_target");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud_source, 250, 0, 0);// 原点云为红色
    viewer.addPointCloud (cloud_source, red, "cloud_source");
    // 键盘回调函数
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) nullptr);

    // 最终可视化的时候会看到，每按一次空格键，红色的原点云会逐渐逼近白色的目标点云
    while (!viewer.wasStopped()){
        viewer.spinOnce ();

        // if you pressed "space"
        if (next_iteration){
            icp.align (*cloud_source);  // 每一次循环，只迭代一次

            if (icp.hasConverged ()){
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
                std::cout << "\nICP transformation " << ++iterations << " : cloud_source -> cloud_target" << std::endl;

                ///得到的旋转矩阵是：target = T * source中的变换矩阵
                Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation ().cast<double>();
                std::cout << transformation_matrix << std::endl;

                viewer.updatePointCloud (cloud_source, red, "cloud_source");
            }
            else{
                std::cerr << "\nICP has not converged.\n" << std::endl;
            }
        }
        next_iteration = false;
    }
}

int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);  // 原点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);  // 目标点云

    std::string name_1 = "/home/chen/CLionProjects/CV_Tools/pcl/PCL_Test/data/000011_1.pcd" ;
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_1, *cloud_source);

    //std::string name_2 = "/home/chen/CLionProjects/CV_Tools/pcl/PCL_Test/data/000014_1.pcd" ;
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_2, *cloud_target);

    ///构造旋转矩阵
    Eigen::Vector3d euler(M_PI/4,0,0);//yaw pitch roll
    Eigen::Matrix3d rotation_mat ;
    rotation_mat = Eigen::AngleAxis(euler[0],Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis(euler[1],Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis(euler[2],Eigen::Vector3d::UnitX());
    ///构造平移向量
    Eigen::Vector3d translate(1.,0,0);

    cout<<"raw R:\n"<<rotation_mat<<endl;
    cout<<"raw t:\n"<<translate<<endl;

    ///构造target矩阵
    cloud_target->points.reserve(cloud_source->points.size());
    for(const auto &pt:cloud_source->points){
        Eigen::Vector3d pt_eigen = rotation_mat * Eigen::Vector3d(pt.x,pt.y,pt.z) + translate;
        pcl::PointXYZRGB pt_target = pt;
        pt_target.x = pt_eigen.x();
        pt_target.y = pt_eigen.y();
        pt_target.z = pt_eigen.z();
        cloud_target->push_back(pt_target);
    }

    std::cout << "[Cloud 1] Number of Points = " << cloud_source->points.size () << std::endl;
    std::cout << "[Cloud 2] Number of Points = " << cloud_target->points.size () << std::endl;

    remove_nan(cloud_source, cloud_target); // 点云中有些点是无效的，要将其去除，要不然 ICP 会报错

    icp_function(cloud_source, cloud_target);

    return 0;
}