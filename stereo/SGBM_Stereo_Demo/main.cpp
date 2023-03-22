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


void StereoCalculatePointCloud(){
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
}



void LKFlow(){
    string left_path="/home/chen/datasets/kitti/tracking/data_tracking_image_2/training/image_02/0003/000000.png";
    string right_path="/home/chen/datasets/kitti/tracking/data_tracking_image_3/training/image_03/0003/000000.png";

    cv::Mat left_img = cv::imread(left_path);
    cv::Mat right_img = cv::imread(right_path);

    cv::Mat left_gray,right_gray;
    cv::cvtColor(left_img,left_gray,CV_BGR2GRAY);
    cv::cvtColor(right_img,right_gray,CV_BGR2GRAY);

    ///特征检测
    vector<cv::Point_<float>> n_pts;
    int max_detect_num=200;
    cv::goodFeaturesToTrack(left_gray, n_pts, max_detect_num, 0.01, 15);

    ///光流跟踪
    std::vector<uchar> status;
    std::vector<float> err;
    vector<cv::Point_<float>> right_pts;
    cv::calcOpticalFlowPyrLK(left_gray, right_gray, n_pts, right_pts, status, err,
                             cv::Size(21, 21), 3);

    cv::Mat show=right_img;
    int N=n_pts.size();
    for(int i=0;i<N;++i){
        if(status[i]){
            cv::circle(show,right_pts[i],2,cv::Scalar(255),-1);
        }
    }

    cout<<N<<endl;

    cv::imshow("test",show);
    cv::waitKey(0);
}



enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };

void calDispWithSGBM(cv::Mat imgL, cv::Mat imgR, cv::Mat &imgDisparity8U)
{
    cv::Size imgSize = imgL.size();
    int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);

    sgbm->setPreFilterCap(63);

    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = imgL.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    int alg = STEREO_SGBM;
    if (alg == STEREO_HH)
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
    else if (alg == STEREO_SGBM)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    else if (alg == STEREO_3WAY)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat imgDisparity16S = cv::Mat(imgL.rows, imgL.cols, CV_16S);
    sgbm->compute(imgL, imgR, imgDisparity16S);

    //--Display it as a CV_8UC1 image：16位有符号转为8位无符号
    imgDisparity16S.convertTo(imgDisparity8U, CV_8U, 255 / (numberOfDisparities*16.));
}


void SGBM_KITTI_Test(){
    //--读取图像
    string left_path="/home/chen/datasets/kitti/tracking/data_tracking_image_2/training/image_02/0003/000000.png";
    string right_path="/home/chen/datasets/kitti/tracking/data_tracking_image_3/training/image_03/0003/000000.png";

    cv::Mat imgLeft = cv::imread(left_path);
    cv::Mat imgRight = cv::imread(right_path);

    cv::Mat imgL = cv::imread(left_path, 0);
    cv::Mat imgR = cv::imread(right_path, 0);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9,
                                                          8 * 9 * 9, 32 * 9 * 9,1, 63,
                                                          10, 100, 32
    );    // 调用OpenCv中的SGBM算法，用于计算左右图像的视差
            cv::Mat disparity_sgbm, disparity;
            sgbm->compute(imgL, imgR, disparity_sgbm);   //将视差的计算结果放入disparity_sgbm矩阵中
            disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f); //将矩阵disparity_sgbm转换为括号中的格式(32位空间的单精度浮点型矩阵)

            cv::imshow("test",disparity_sgbm);
            cv::waitKey(0);


            cv::Mat disp=disparity;

            ///KITTI
            double fx=721.5377;
            double fy=721.5377;
            double cx=609.5593;
            double cy=172.8540;
            double bf=387.5744/fx;

            pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

            for (int v = 0; v < imgLeft.rows; v++){
                for (int u = 0; u < imgLeft.cols; u++) {
                    if (disp.at<float>(v, u) <= 0.0 || disp.at<float>(v, u) >= 96.0) continue;
                    // 根据双目模型计算 point 的位置
                    double x = (u - cx) / fx;
                    double y = (v - cy) / fy;
                    double depth = fx * bf / (disp.at<float>(v, u));
                    auto pixel = imgLeft.at<cv::Vec3b>(v,u);
                    PointT p(pixel[0],pixel[1],pixel[2]);
                    p.x = x * depth;
                    p.y = y * depth;
                    p.z = depth;
                    cloud->push_back(p);
                    if(v==u){
                        cout<<disp.at<float>(v, u)<<endl;
                    }
                }
            }

            cout<<cloud->points.size()<<endl;

            pcl::visualization::CloudViewer viewer ("test");
            viewer.showCloud(cloud);
            while (!viewer.wasStopped()){ };

}



void SGBM_ZED_Test(){

    //string left_path="/home/chen/datasets/MyData/ZED_data/un_cam0/road_3/005700.png";
    //string right_path="/home/chen/datasets/MyData/ZED_data/un_cam1/road_3/005700.png";

    string left_path="/home/chen/datasets/MyData/ZED_data/cam0/road_1/003014.png";
    string right_path="/home/chen/datasets/MyData/ZED_data/cam1/road_1/003014.png";

    cv::Mat imgLeft = cv::imread(left_path);
    cv::Mat imgRight = cv::imread(right_path);


    cv::Mat imgL = cv::imread(left_path, 0);
    cv::Mat imgR = cv::imread(right_path, 0);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9,
                                                          8 * 9 * 9, 32 * 9 * 9,1, 63, 10, 100, 32);    // 调用OpenCv中的SGBM算法，用于计算左右图像的视差
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(imgL, imgR, disparity_sgbm);   //将视差的计算结果放入disparity_sgbm矩阵中
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f); //将矩阵disparity_sgbm转换为括号中的格式(32位空间的单精度浮点型矩阵)

    cv::imshow("test",disparity_sgbm);
    cv::waitKey(0);

    cv::imwrite("disp.png",disparity_sgbm);

    cv::Mat disp=disparity;

    ///ZED un
    /*double fx=5.7817315673828125e+02;
    double fy=6.6596881103515625e+02;
    double cx=6.7666424560546875e+02;
    double cy=3.6173339843750000e+02;*/

    double fx=701.406049185687;
    double fy=700.7199834541797;
    double cx=663.9703743586792;
    double cy=362.02045484177154;

    double bf=0.12;

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    for (int v = 0; v < imgLeft.rows; v++){
        for (int u = 0; u < imgLeft.cols; u++) {
            if (disp.at<float>(v, u) <= 0.0 || disp.at<float>(v, u) >= 96.0) continue;
            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * bf / (disp.at<float>(v, u));
            auto pixel = imgLeft.at<cv::Vec3b>(v,u);
            PointT p(pixel[0],pixel[1],pixel[2]);
            p.x = x * depth;
            p.y = y * depth;
            p.z = depth;
            cloud->push_back(p);
            if(v==u){
                cout<<disp.at<float>(v, u)<<endl;
            }
        }
    }

    cout<<cloud->points.size()<<endl;

    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };

}


int main(int argc, char **argv)
{

    //SGBM_ZED_Test();

    SGBM_KITTI_Test();

    return 0;
}
