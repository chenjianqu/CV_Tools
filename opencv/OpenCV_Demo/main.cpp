#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/xfeatures2d.hpp>

/*#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaoptflow.hpp>*/

#include <chrono>


using namespace std;
using namespace cv;


/*
void flow_track_test(cv::Mat &img1,cv::Mat img2)
{
    //cv::cuda::GpuMat src,dst;
    //src.upload(img1);
    //auto detector=cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1,200,0.01,20);

    vector<cv::Point_<float>> new_pts;
    detector->detect(img1,new_pts);
}
*/


void BasicTest()
{
    Mat leftImg=imread("left.jpg",0);
    Mat rightImg=imread("right.jpg",0);

    imshow ( "leftImg", leftImg);
    imshow ( "rightImg", rightImg);
    waitKey ( 0 );

    destroyAllWindows();
}


void VideoTest()
{
    cv:: VideoCapture cap = cv::VideoCapture(1);
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cv::Mat frame, img;
    cap.read(frame);
    cout<<frame.size<<endl;

    while(cap.isOpened()){
        clock_t start = clock();
        cap.read(frame);
        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }

        cv::imshow("Test", frame);
        if(cv::waitKey(1)== 27) break;
    }
}


void VideoStereoTest()
{
    cv:: VideoCapture cap0 = cv::VideoCapture(0);
    cap0.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap0.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    cv:: VideoCapture cap1 = cv::VideoCapture(1);
    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    cv::Mat frame0, frame1;
    cap0.read(frame0);
    cout<<frame0.size<<endl;

    cap1.read(frame1);
    cout<<frame1.size<<endl;

    while(cap0.isOpened() && cap1.isOpened()){
        cap0.read(frame0);
        cap1.read(frame1);
        if(frame0.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }

        cv::imshow("Test0", frame0);
        cv::imshow("Test1", frame1);
        if(cv::waitKey(1)== 27) break;
    }
}



void VideoUVCTest()
{
    cv:: VideoCapture cap = cv::VideoCapture(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cv::Mat frame, img;
    cap.read(frame);
    cout<<frame.size<<endl;

    cv::Rect rect0(0,0,1280,720);
    cv::Rect rect1(1280,0,1280,720);

    while(cap.isOpened()){

        auto start = std::chrono::steady_clock::now();

        cap.read(frame);

        cv::Mat img0=frame(rect0);
        cv::Mat img1=frame(rect1);

        if(frame.empty()){
            std::cout << "Read frame failed!" << std::endl;
            break;
        }


        //cv::imshow("Test", frame);
        //if(cv::waitKey(1)== 27) break;

        cout<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()<<endl;
    }
}



void ReadWriteYamlTest(){
    string config_path = "/home/chen/CLionProjects/CV_Tools/OpenCV_Test/viode.yaml";
    cv::FileStorage fs(config_path,
                       cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        throw std::runtime_error(std::string("ERROR: Wrong path to settings:" + config_path));
    }

    fs<<"dataset_sequence"<<string("Test").data();


    fs.release();
}



void KTreeTest(){

    cv::Point2d p0(0,0);
    cv::Point2d p1(1,0);
    cv::Point2d p2(0,1);
    cv::Point2d p3(1,1);

    vector<cv::Point2d> cv_points2d = {p0,p1,p2,p3};

    cv::Mat mat_points = cv::Mat(cv_points2d);//shape为

    cout<<mat_points.rows<<" "<<mat_points.cols<<" "<<mat_points.channels()<<" "<< endl;

    mat_points = mat_points.reshape(1);
    cout<<mat_points.rows<<" "<<mat_points.cols<<" "<<mat_points.channels()<<" "<< endl;

    cout<<mat_points.type()<<endl;

    mat_points.convertTo(mat_points,CV_32F);///cv::Point2d的数据类型为64F

    cv::flann::KDTreeIndexParams indexParams(2); //The number of parallel kd-trees to use. 范围： [1-16]
    cv::flann::Index kdtree(mat_points, indexParams); //kd树索引建立完毕

    ///根据查找数量来进行搜索
    unsigned queryNum = 2;//用于设置返回邻近点的个数
    vector<float> vecQuery(2);//存放 查询点 的容器（本例都是vector类型）
    vecQuery[0] = 0.5; //查询点x坐标
    vecQuery[1] = 0; //查询点y坐标

    vector<int> vecIndex(queryNum);//存放返回的点索引
    vector<float> vecDist(queryNum);//存放距离
    flann::SearchParams params(32);//设置knnSearch搜索参数

    kdtree.knnSearch(vecQuery, vecIndex, vecDist, queryNum, params);

    for(int i=0;i<vecIndex.size();++i){
        cout<<vecIndex[i]<<" "<<vecDist[i]<<endl;
    }

    ///根据查找半径来进行搜索
    vecQuery[0] = 0.1; //查询点x坐标
    vecQuery[1] = 0; //查询点y坐标

    vecIndex.clear();
    vecDist.clear();
    int num_find = kdtree.radiusSearch(vecQuery,vecIndex,vecDist,1,5);

    cout<<num_find<<endl;

    for(int i=0;i<vecIndex.size();++i){
        cout<<vecIndex[i]<<" "<<vecDist[i]<<endl;
    }

}


void MatchGMS_Test(){
    Mat img_1 = imread("/media/cjq/新加卷/datasets/YT_dataset/YT_gaosus/2022-11-24-15-22-31/sfm/0_958/images/"
                       "1669274554.826000.png");
    Mat img_2 = imread("/media/cjq/新加卷/datasets/YT_dataset/YT_gaosus/2022-11-24-15-22-31/sfm/0_958/images/"
                       "1669274555.576000.png");
    Ptr<Feature2D> sift = SIFT::create(2048);
    //Ptr<Feature2D> surf = cv::xfeatures2d::SURF::create();
    //Ptr<ORB> orb = ORB::create(5000);
    //orb->setFastThreshold(0);

    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1,descriptors_2;

    sift->detectAndCompute(img_1,cv::Mat(),keypoints_1,descriptors_1);
    sift->detectAndCompute(img_2,cv::Mat(),keypoints_2,descriptors_2);

    cout<<descriptors_1.rows<<" "<<descriptors_1.cols<<" "<<descriptors_1.type()<<endl;

    //orb->detectAndCompute(img_1, Mat(),keypoints_1, descriptors_1);
    //orb->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);

    //orb ->detect(img_1, keypoints_1);
    //orb ->compute(img_1, keypoints_1, descriptors_1);

    //orb->detect(img_2, keypoints_2);
    //orb->compute(img_2, keypoints_2, descriptors_2);

    Mat ShowKeypoints1, ShowKeypoints2;
    drawKeypoints(img_1, keypoints_1, ShowKeypoints1);
    drawKeypoints(img_2, keypoints_2, ShowKeypoints2);
    imshow("Result_1", ShowKeypoints1);
    imshow("Result_2", ShowKeypoints2);

    vector<DMatch> matchesAll, matchesGMS;
    BFMatcher matcher(NORM_L1);
    matcher.match(descriptors_1, descriptors_2, matchesAll);

    Mat finalMatches_1;
    //drawMatches(img_1, keypoints_1, img_2, keypoints_2, matchesAll, finalMatches_1, Scalar::all(-1), Scalar::all(-1),
    //            std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //imshow("Matches bf", finalMatches_1);


    cout << "matchesAll: " << matchesAll.size() << endl;
    cv::xfeatures2d::matchGMS(img_1.size(), img_2.size(), keypoints_1, keypoints_2,
                              matchesAll, matchesGMS,false,true,5);
    std::cout << "matchesGMS: " << matchesGMS.size() << std::endl;

    Mat finalMatches;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matchesGMS, finalMatches, Scalar::all(-1), Scalar::all(-1),
                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("Matches GMS", finalMatches);
    imwrite("MatchesGMS.jpg", finalMatches);
    waitKey(0);

}









int main ( int argc, char** argv )
{
    cout << "OpenCV version : " << CV_VERSION << endl;

    //VideoTest();

    //VideoUVCTest();

    //ReadWriteYamlTest();

    //KTreeTest();

    MatchGMS_Test();


    return 0;
}