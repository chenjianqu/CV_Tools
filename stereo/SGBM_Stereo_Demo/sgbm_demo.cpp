#include <opencv2/opencv.hpp>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include "def.h"
#include "io_utils.h"

using namespace std;

using PointT = pcl::PointXYZRGB;



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


int main(int argc, char **argv)
{
    if(argc!=4){
        cerr<<"usage:./sgbm_demo ${cam0_dir} ${cam1_dir} ${output_dir}"<<endl;
        return -1;
    }

    fs::path left_image_dir=argv[1];
    fs::path right_image_dir=argv[2];
    fs::path output_dir=argv[3];

    if(!fs::exists(left_image_dir)){
        cerr<<left_image_dir<<" do not exists"<<endl;
        return -1;
    }
    if(!fs::exists(right_image_dir)){
        cerr<<right_image_dir<<" do not exists"<<endl;
        return -1;
    }
    if(!CheckIsDir(output_dir)){
        cerr<<output_dir<<" is not a dir"<<endl;
        return -1;
    }

    vector<fs::path> left_names = GetDirectoryFileNames(left_image_dir);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,
                                                          96, 9, 8 * 9 * 9, 32 * 9 * 9,
                                                          1, 63, 10, 100, 32
                                                          );// 调用OpenCv中的SGBM算法，用于计算左右图像的视差

    TicToc t_all,t_step;

    for(auto &name:left_names){
        t_all.Tic();

        fs::path left_path = left_image_dir / name;
        if(!fs::exists(left_path)){
            cerr<<left_path<<" is not exists"<<endl;
            break;
        }
        fs::path right_path = right_image_dir / name;
        if(!fs::exists(right_path)){
            cerr<<right_path<<" is not exists"<<endl;
            break;
        }

        cout<<left_path<<endl;

        t_step.Tic();
        cv::Mat left_img = cv::imread(left_path.string());
        cv::Mat right_img = cv::imread(right_path.string(), -1);

        cout<<"imread:"<<t_step.TocThenTic()<<" ms"<<endl<<endl;

        cv::Mat disparity_sgbm, disparity;
        sgbm->compute(left_img, right_img, disparity_sgbm);   //将视差的计算结果放入disparity_sgbm矩阵中
        disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f); //将矩阵disparity_sgbm转换为括号中的格式(32位空间的单精度浮点型矩阵)

        cout<<"sgbm->compute:"<<t_step.TocThenTic()<<" ms"<<endl<<endl;

        cv::imwrite((output_dir/name).string(),disparity_sgbm);
        cout<<"imwrite:"<<t_step.TocThenTic()<<" ms"<<endl<<endl;

        cv::imshow("disp",disparity);
        cv::waitKey(1);


        cout<<"t_all:"<<t_all.Toc()<<" ms"<<endl<<endl;
    }


    return 0;
}

