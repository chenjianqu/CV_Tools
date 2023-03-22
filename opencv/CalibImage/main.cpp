#include <iostream>
#include <filesystem>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "def.h"
#include "io_utils.h"


namespace fs = std::filesystem;


/**
 * 针孔相机
 */
class PinholeCalib{
public:
    PinholeCalib(int img_width_,int img_height_,float fx_,float fy_,float cx_,float cy_):
    img_width(img_width_),img_height(img_height_),
    fx(fx_),fy(fy_),cx(cx_),cy(cy_){}


    explicit PinholeCalib(const std::string &file_name){
        cv::FileStorage fs(file_name, cv::FileStorage::READ);
        if(!fs.isOpened()){
            throw std::runtime_error(std::string("ERROR: Wrong path to settings:"+file_name));
        }
        fs["camera_name"]>>cam_name;
        fs["image_width"]>>img_width;
        fs["image_height"]>>img_height;

        cv::FileNode n = fs["distortion_parameters"];
        k1 = static_cast<double>(n["k1"]);
        k2 = static_cast<double>(n["k2"]);
        p1 = static_cast<double>(n["p1"]);
        p2 = static_cast<double>(n["p2"]);

        n = fs["projection_parameters"];
        fx = static_cast<double>(n["fx"]);
        fy = static_cast<double>(n["fy"]);
        cx = static_cast<double>(n["cx"]);
        cy = static_cast<double>(n["cy"]);

        fs.release();
    }


    void writeToYamlFile(const std::string& filename) const
    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);

        fs << "model_type" << "PINHOLE";
        fs << "camera_name" << cam_name;
        fs << "image_width" << img_width;
        fs << "image_height" << img_height;

        // radial distortion: k1, k2
        // tangential distortion: p1, p2
        fs << "distortion_parameters";
        fs << "{" << "k1" << k1
        << "k2" << k2
        << "p1" << p1
        << "p2" << p2 << "}";

        // projection: fx, fy, cx, cy
        fs << "projection_parameters";
        fs << "{" << "fx" << fx
        << "fy" << fy
        << "cx" << cx
        << "cy" << cy << "}";

        fs.release();
    }


    /**
     * \brief Apply distortion to input point (from the normalised plane)
     *
     * \param p_u undistorted coordinates of point on the normalised plane
     * \return to obtain the distorted point: p_d = p_u + d_u
     */
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
    {
        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
        p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    }


    /**
     * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
     *
     * \param P 3D point coordinates
     * \param p return value, contains the image point coordinates
     */
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
    {
        Eigen::Vector2d p_u, p_d;
        // Project points to the normalised plane
        p_u << P(0) / P(2), P(1) / P(2);

        if (m_noDistortion){
            p_d = p_u;
        }
        else{
            // Apply distortion
            Eigen::Vector2d d_u;
            distortion(p_u, d_u);
            p_d = p_u + d_u;
        }

        // Apply generalised projection matrix
        p << fx * p_d(0) + cx,
        fy * p_d(1) + cy;
    }

    /**
     * 得到矫正映射图，并得到新的内参。
     */
    void InitUndistortRectifyMap(){
        ///注意，执行矫正了，图像的内参改变了
        const double alpha=0;//值为0,损失最多的像素，没有黑色边框；值为1时，所有像素保留，但存在黑色边框。
        cv::Mat new_K0 = cv::getOptimalNewCameraMatrix(
                GetK(),GetD(),
                GetImageSize(),alpha,
                GetImageSize());

        cv::initUndistortRectifyMap(
                GetK(),GetD(),cv::Mat(),new_K0,
                GetImageSize(),CV_16SC2,undist_map1,undist_map2);

        SetIntrinsics(new_K0.at<double>(0,0),new_K0.at<double>(1,1),
                      new_K0.at<double>(0,2),new_K0.at<double>(1,2));
        SetDistortionParameters(0,0,0,0);
    }


    void SetDistortionParameters(float k1_,float k2_,float p1_,float p2_){
        k1 = k1_;
        k2 = k2_;
        p1 = p1_;
        p2 = p2_;
    }

    void SetIntrinsics(float fx_,float fy_,float cx_,float cy_){
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    [[nodiscard]] cv::Mat GetK() const{
        return cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1;
    }

    [[nodiscard]] cv::Mat GetD() const{
        return cv::Mat_<double>(5,1)<<k1,k2,0,p1,p2;
    }

    [[nodiscard]] cv::Size GetImageSize() const{
        return {img_width,img_height};
    }

private:
    int img_width{},img_height{};
    float fx,fy,cx,cy;
    float k1{0},k2{0},p1{0},p2{0};

    bool m_noDistortion{false};

    std::string cam_name;

public:
    cv::Mat undist_map1, undist_map2;

};



int main(int argc,char** argv) {

    if(argc<3 || argc>6){
        cerr<<"usage: ./CalibImageTest ${left_cam_param} ${right_cam_param} ${left_img_dir} ${right_img_dir} ${output_path}"<<endl;
        cout<<"or"<<endl;
        cerr<<"usage: ./CalibImageTest ${left_cam_param} ${right_cam_param} ${left_img_dir} ${right_img_dir}"<<endl;
        cout<<"or"<<endl;
        cerr<<"usage: ./CalibImageTest ${cam_param} ${img_dir} ${output_path}"<<endl;
        cout<<"or"<<endl;
        cerr<<"usage: ./CalibImageTest ${cam_param} ${img_dir} "<<endl;
        return -1;
    }

    bool is_stereo = argc>=5;
    bool is_write_img = argc==4 || argc==6;

    fs::path left_cam_param_path;
    fs::path left_img_dir;
    fs::path right_cam_param_path;
    fs::path right_img_dir;
    fs::path output_dir;

    if(argc==3){
        left_cam_param_path = argv[1];
        left_img_dir = argv[2];
    }
    else if(argc==4){
        left_cam_param_path = argv[1];
        left_img_dir = argv[2];
        output_dir = argv[3];
    }
    else if(argc==5){
        left_cam_param_path = argv[1];
        right_cam_param_path = argv[2];
        left_img_dir = argv[3];
        right_img_dir = argv[4];
    }
    else if(argc==6){
        left_cam_param_path = argv[1];
        right_cam_param_path = argv[2];
        left_img_dir = argv[3];
        right_img_dir = argv[4];
        output_dir = argv[5];
    }

    if(!left_cam_param_path.empty() && !fs::exists(left_cam_param_path)){
        cerr<<left_cam_param_path<<" do not exists"<<endl;
        return -1;
    }
    if(!right_cam_param_path.empty() && !fs::exists(right_cam_param_path)){
        cerr<<right_cam_param_path<<" do not exists"<<endl;
        return -1;
    }
    if(!left_img_dir.empty() && !CheckIsDir(left_img_dir)){
        cerr<<left_img_dir<<" is not a dir"<<endl;
        return -1;
    }
    if(!right_img_dir.empty() && !CheckIsDir(right_img_dir)){
        cerr<<right_img_dir<<" is not a dir"<<endl;
        return -1;
    }
    if(!output_dir.empty() && !CheckIsDir(output_dir)){
        cerr<<output_dir<<" is not a dir"<<endl;
        return -1;
    }

    ///创建输出文件夹
    fs::path output_left_dir;
    fs::path output_right_dir;

    if(is_write_img){
        if(is_stereo){
            output_left_dir = output_dir / "cam0";
            output_right_dir = output_dir / "cam1";

            if(!fs::exists(output_left_dir)){
                int isCreate = mkdir(output_left_dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
            }
            if(!fs::exists(output_right_dir)){
                int isCreate = mkdir(output_right_dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
            }
        }
        else{
            output_left_dir = output_dir;
            if(!fs::exists(output_left_dir)){
                int isCreate = mkdir(output_left_dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
            }
        }
    }

    //左相机内参和畸变系数
    std::shared_ptr<PinholeCalib> left_cam = std::make_shared<PinholeCalib>(left_cam_param_path.string());
    std::shared_ptr<PinholeCalib> right_cam;

    left_cam->InitUndistortRectifyMap();
    left_cam->writeToYamlFile("cam0.yaml");

    if(is_stereo){
        right_cam = std::make_shared<PinholeCalib>(right_cam_param_path.string());
        right_cam->InitUndistortRectifyMap();
        right_cam->writeToYamlFile("cam1.yaml");
    }

    vector<fs::path> left_names = GetDirectoryFileNames(left_img_dir);

    int names_size = left_names.size();
    for(int i=0;i<names_size;++i){
        fs::path left_path = left_img_dir / left_names[i];
        cout<<left_path<<endl;
        cv::Mat img0_raw = cv::imread(left_path.string());

        cv::Mat un_image0;
        cv::remap(img0_raw, un_image0, left_cam->undist_map1,left_cam->undist_map2, CV_INTER_LINEAR);

        if(is_write_img){
            fs::path left_output_path = output_left_dir / left_names[i];
            cv::imwrite(left_output_path.string(),un_image0);
        }

        cv::Mat img_show;

        if(is_stereo){
            cv::Mat un_image1;
            fs::path right_path = right_img_dir / left_names[i];
            cout<<right_path<<endl;
            cv::Mat img1_raw = cv::imread(right_path.string());
            cv::remap(img1_raw, un_image1, left_cam->undist_map1,left_cam->undist_map2, CV_INTER_LINEAR);

            if(is_write_img){
                fs::path right_output_path = output_right_dir / left_names[i];
                cv::imwrite(right_output_path.string(),un_image1);
            }

            ///可视化
            cv::Mat img_show_raw,img_show_un;
            cv::hconcat(vector<cv::Mat>{img0_raw,img1_raw},img_show_raw);
            cv::hconcat(vector<cv::Mat>{un_image0,un_image1},img_show_un);
            cv::vconcat(vector<cv::Mat>{img_show_raw,img_show_un},img_show);
        }
        else{
            cv::vconcat(vector<cv::Mat>{img0_raw,un_image0},img_show);
        }

        cv::resize(img_show,img_show,cv::Size(),0.5,0.5);

        cv::imshow("calib",img_show);
        cv::waitKey(1);
    }



    return 0;
}
