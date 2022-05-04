//
// Created by chen on 2022/5/2.
//

#ifndef FCOS_3DBOX_VISUALIZATION_H
#define FCOS_3DBOX_VISUALIZATION_H

#include <Eigen/Core>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>


visualization_msgs::Marker BuildLineStripMarker(Eigen::Matrix<double,8,3> &corners,int id);

visualization_msgs::Marker BuildTextMarker(const std::string& text,const Eigen::Vector3d &pos,int id);

visualization_msgs::Marker BuildSphereMarker(const Eigen::Vector3d &pos,double scale,int id);



#endif //FCOS_3DBOX_VISUALIZATION_H
