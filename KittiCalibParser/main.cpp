#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>

#include <Eigen/Dense>

using namespace std;

using Mat34d = Eigen::Matrix<double,3,4,Eigen::RowMajor>;
using Mat4d = Eigen::Matrix<double,4,4,Eigen::RowMajor>;

Mat34d P0,P1,P2,P3;
Eigen::Matrix3d R_rect;
Mat4d Tr_velo_cam,Tr_imu_velo;

double baseline_1;
double baseline_3;

void split(const std::string& source, std::vector<std::string>& tokens, const string& delimiters = " ") {
    std::regex re(delimiters);
    std::copy(std::sregex_token_iterator(source.begin(), source.end(),re,-1),
              std::sregex_token_iterator(),
              std::back_inserter(tokens));
}

void ReadCalibFile(const string &path){
    ifstream fp(path); //定义声明一个ifstream对象，指定文件路径
    string line;
    while (getline(fp,line)){ //循环读取每行数据
        vector<string> tokens;
        split(line,tokens," ");
        if(tokens[0]=="P0:"){
            vector<double> data(12);
            for(int i=0;i<12;++i)
                data[i] = std::stod(tokens[i+1]);
            P0 = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data.data(), 3, 4);
        }
        else if(tokens[0]=="P1:"){
            vector<double> data(12);
            for(int i=0;i<12;++i)
                data[i] = std::stod(tokens[i+1]);
            P1 = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data.data(), 3, 4);
        }
        else if(tokens[0]=="P2:"){
            vector<double> data(12);
            for(int i=0;i<12;++i)
                data[i] = std::stod(tokens[i+1]);
            P2 = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data.data(), 3, 4);
        }
        else if(tokens[0]=="P3:"){
            vector<double> data(12);
            for(int i=0;i<12;++i)
                data[i] = std::stod(tokens[i+1]);
            P3 = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data.data(), 3, 4);
        }
        else if(tokens[0]=="R_rect"){
            vector<double> data(9);
            for(int i=0;i<9;++i)
                data[i] = std::stod(tokens[i+1]);
            R_rect = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(data.data(), 3, 3);
        }
        else if(tokens[0]=="Tr_velo_cam"){
            vector<double> data(12);
            for(int i=0;i<12;++i)
                data[i] = std::stod(tokens[i+1]);
            Tr_velo_cam = Mat4d::Identity();
            Tr_velo_cam.topLeftCorner(3,4) = Eigen::Map<Mat34d>(data.data(), 3, 4);
        }
        else if(tokens[0]=="Tr_imu_velo"){
            vector<double> data(12);
            for(int i=0;i<12;++i)
                data[i] = std::stod(tokens[i+1]);

            Tr_imu_velo = Mat4d::Identity();
            Tr_imu_velo.topLeftCorner(3,4) = Eigen::Map<Mat34d>(data.data(), 3, 4);
        }
    }
    fp.close();

}



int main(){
    const string calib_file = "/home/chen/CLionProjects/CV_Tools/KittiCalibParser/data/0000.txt";
    ReadCalibFile(calib_file);

    double fx1 = P1(0,0);
    baseline_1 = P1(0,3) / (-fx1);

    double fx2 = P2(0,0);
    double baseline_2 = P2(0,3) / (-fx2);

    double fx3 = P3(0,0);
    baseline_3 = P3(0,3) / (-fx3);

    double baseline32 = baseline_3 - baseline_2;

    cout<<"baseline_1:"<<baseline_1<<endl;
    cout<<"baseline_2:"<<baseline_2<<endl;
    cout<<"baseline_3:"<<baseline_3<<endl;

    cout<<"baseline_23:"<<baseline_3-baseline_2<<endl;


    cout<<"P0"<<endl<<P0<<endl;
    cout<<"P1"<<endl<<P1<<endl;
    cout<<"P2"<<endl<<P2<<endl;
    cout<<"P3"<<endl<<P3<<endl;

    cout<<"R_rect"<<endl<<R_rect<<endl;
    cout<<"Tr_velo_cam"<<endl<<Tr_velo_cam<<endl;
    cout<<"Tr_imu_velo"<<endl<<Tr_imu_velo<<endl;


    Mat4d T_imu_c0 =Tr_imu_velo * Tr_velo_cam;


    Mat4d T_c0_c2 = Mat4d::Identity();
    T_c0_c2(0,3) = -baseline32;
    Mat4d T_imu_c2 = T_imu_c0*T_c0_c2;
    Mat4d T_ic = T_imu_c2.inverse();

    cout<<"T_ic:\n"<<T_ic<<endl;

    return 0;
}