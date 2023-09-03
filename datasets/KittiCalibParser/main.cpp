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


void ParseCalibTest(){
    const string calib_file = "/home/chen/datasets/kitti/tracking/data_tracking_calib/training/calib/0003.txt";
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
}

vector<vector<double>> ReadOxtsData(const string &path){
    vector<vector<double>> data;
    ifstream fp(path); //定义声明一个ifstream对象，指定文件路径
    string line;
    vector<double> tokens;
    while (getline(fp,line)){ //循环读取每行数据
        stringstream text_stream(line);
        string item;
        tokens.clear();
        while (std::getline(text_stream, item, ' ')) {
            tokens.push_back(stod(item));
        }
        data.push_back(tokens);
    }
    fp.close();

    return data;
}

/**
 * converts lat/lon coordinates to mercator coordinates using mercator scale
 * @param lat
 * @param lon
 * @param scale
 * @return
 */
std::pair<double,double> LatLonToMercator(double lat,double lon,double scale){
    constexpr double er = 6378137;
    double mx = scale * lon * M_PI * er /180.;
    double my = scale * er * log(tan((90+lat)*M_PI/360));
    return {mx,my};
}

/**
 % converts a list of oxts measurements into metric poses,
% starting at (0,0,0) meters, OXTS coordinates are defined as
% x = forward, y = right, z = down (see OXTS RT3000 user manual)
% afterwards, pose{i} contains the transformation which takes a
% 3D point in the i'th frame and projects it into the oxts
% coordinates of the first frame.
 */
void ParseOxtsTest(){
    string data_path="/home/chen/CLionProjects/CV_Tools/KittiCalibParser/data/0003.txt";

    auto data = ReadOxtsData(data_path);
    if(data.empty()){
        cerr<<"data.empty()"<<endl;
        std::terminate();
    }

    double scale = cos(data[0][0] * M_PI / 180.0);

    vector<Mat4d> pose;
    Mat4d Tr_0_inv;
    bool is_init=false;

    for(auto &row:data){
        //for(double d:row) cout<<d<<" ";
        //cout<<endl;
        cout<<row.size()<<endl;
        Eigen::Vector3d t;
        std::tie(t.x(),t.y()) = LatLonToMercator(row[0],row[1],scale);
        t.z() = row[2];

        //cout<<t.transpose()<<endl;

        double rx = row[3];
        double ry = row[4];
        double rz = row[5];

        Eigen::Matrix3d Rx,Ry,Rz;
        Rx << 1, 0, 0,
        0,cos(rx), -sin(rx),
        0, sin(rx), cos(rx); // base => nav  (level oxts => rotated oxts)
        Ry<<cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry); // base => nav  (level oxts => rotated oxts)

        Rz<<cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1; // base => nav  (level oxts => rotated oxts)
        Eigen::Matrix3d R  = Rz*Ry*Rx;

        if(!is_init){
            is_init=true;
            Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
            T_inv.topLeftCorner(3,3) = R;
            T_inv.block<3,1>(0,3) = t;
            cout<<T_inv<<endl;
            Tr_0_inv = T_inv.inverse();
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.topLeftCorner(3,3) = R;
        T.block<3,1>(0,3) = t;
        T = Tr_0_inv * T;
        pose.emplace_back(T);

        cout<<T<<endl;

    }

}


int main(){
    ParseOxtsTest();

    return 0;
}