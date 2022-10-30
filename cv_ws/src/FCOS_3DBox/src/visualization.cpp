//
// Created by chen on 2022/5/2.
//

#include "visualization.h"





using namespace std;



visualization_msgs::Marker BuildLineStripMarker(Eigen::Matrix<double,8,3> &corners,int id){
    visualization_msgs::Marker msg;

    msg.header.frame_id="camera";
    msg.header.stamp=ros::Time::now();
    msg.ns="box_strip";
    msg.action=visualization_msgs::Marker::ADD;
    msg.pose.orientation.w=1.0;

    //暂时使用类别代替这个ID
    msg.id=id;//当存在多个marker时用于标志出来
    //cout<<msg.id<<endl;
    msg.lifetime=ros::Duration(4);//持续时间3s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::LINE_STRIP;//marker的类型
    msg.scale.x=0.01;//线宽
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度

    //设置立方体的八个顶点
    geometry_msgs::Point p[8];
    p[0].x= corners(0,0);p[0].y=corners(0,1);p[0].z=corners(0,2);
    p[1].x= corners(1,0);p[1].y=corners(1,1);p[1].z=corners(1,2);
    p[2].x= corners(2,0);p[2].y=corners(2,1);p[2].z=corners(2,2);
    p[3].x= corners(3,0);p[3].y=corners(3,1);p[3].z=corners(3,2);
    p[4].x= corners(4,0);p[4].y=corners(4,1);p[4].z=corners(4,2);
    p[5].x= corners(5,0);p[5].y=corners(5,1);p[5].z=corners(5,2);
    p[6].x= corners(6,0);p[6].y=corners(6,1);p[6].z=corners(6,2);
    p[7].x= corners(7,0);p[7].y=corners(7,1);p[7].z=corners(7,2);
    /**
             .. code-block:: none

                             front z
                                    /
                                   /
                   p1(x0, y0, z1) + -----------  + p5(x1, y0, z1)
                                 /|            / |
                                / |           /  |
                p0(x0, y0, z0) + ----------- +   + p6(x1, y1, z1)
                               |  /      .   |  /
                               | / origin    | /
                p3(x0, y1, z0) + ----------- + -------> x right
                               |             p7(x1, y1, z0)
                               |
                               v
                        down y
     输入的点序列:p0:0,0,0, p1: 0,0,1,  p2: 0,1,1,  p3: 0,1,0,  p4: 1,0,0,  p5: 1,0,1,  p6: 1,1,1,  p7: 1,1,0;

     */
    msg.points.push_back(p[0]);
    msg.points.push_back(p[1]);
    msg.points.push_back(p[5]);
    msg.points.push_back(p[4]);
    msg.points.push_back(p[0]);
    msg.points.push_back(p[3]);
    msg.points.push_back(p[7]);
    msg.points.push_back(p[4]);
    msg.points.push_back(p[7]);
    msg.points.push_back(p[6]);
    msg.points.push_back(p[5]);
    msg.points.push_back(p[6]);
    msg.points.push_back(p[2]);
    msg.points.push_back(p[1]);
    msg.points.push_back(p[2]);
    msg.points.push_back(p[3]);

    return msg;
}



visualization_msgs::Marker BuildTextMarker(const std::string& text,const Eigen::Vector3d &pos,int id)
{
    visualization_msgs::Marker msg;

    msg.header.frame_id="camera";
    msg.header.stamp=ros::Time::now();
    msg.ns="box_text";
    msg.action=visualization_msgs::Marker::ADD;

    //暂时使用类别代替这个ID
    msg.id=id;//当存在多个marker时用于标志出来
    msg.lifetime=ros::Duration(4);//持续时间4s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::TEXT_VIEW_FACING;//marker的类型
    //    msg.scale.x=0.01;//线宽
    msg.scale.z=0.5;
    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度

    geometry_msgs::Pose pose;
    pose.position.x=pos.x();pose.position.y=pos.y();pose.position.z=pos.z();
    pose.orientation.w=1.0;
    msg.pose=pose;

    msg.text=text;

    return msg;
}


visualization_msgs::Marker BuildSphereMarker(const Eigen::Vector3d &pos,double scale,int id)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id="camera";
    msg.header.stamp=ros::Time::now();
    msg.ns="box_sphere";
    msg.action=visualization_msgs::Marker::ADD;

    //暂时使用类别代替这个ID
    msg.id=id;//当存在多个marker时用于标志出来
    msg.lifetime=ros::Duration(4);//持续时间4s，若为ros::Duration()表示一直持续

    msg.type=visualization_msgs::Marker::SPHERE;//marker的类型
    //    msg.scale.x=0.01;//线宽
    msg.scale.x=scale;
    msg.scale.y=scale;
    msg.scale.z=scale;

    msg.color.r=1.0;msg.color.g=1.0;msg.color.b=1.0;
    msg.color.a=1.0;//不透明度

    geometry_msgs::Pose pose;
    pose.position.x=pos.x();pose.position.y=pos.y();pose.position.z=pos.z();
    pose.orientation.w=1.0;
    msg.pose=pose;

    return msg;
}






