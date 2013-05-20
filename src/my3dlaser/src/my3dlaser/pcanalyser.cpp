/*
 * pcanalyser.cpp
 *
 *  Created on: 17.08.2010
 *      Author: ropra
 */

#include <pluginlib/class_list_macros.h>
//#include <pcl/point_types.h>
#include "my3dlaser/pcanalyser.h"
//#include "my3dlaser/mypclrosfpfh.h"
//#include "my3dlaser/mynormalestimation.h"

//typedef myanalyeserns::PointCloudAnalyser PointCloudAnalyser;

PLUGINLIB_DECLARE_CLASS (my3dlaser, PointCloudAnalyser, myanalyeserns::PointCloudAnalyser, nodelet::Nodelet);
//PLUGINLIB_DECLARE_CLASS (myanalyeserns, MyPclRosFPFH, myanalyeserns::MyPclRosFPFH, nodelet::Nodelet);
//PLUGINLIB_DECLARE_CLASS (myanalyeserns, MyNormalEstimation, myanalyeserns::MyNormalEstimation, nodelet::Nodelet);


