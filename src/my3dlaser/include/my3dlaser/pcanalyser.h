/*
 * pcanalyser.h
 *
 *  Created on: 17.08.2010
 *      Author: ropra
 */

#ifndef PCANALYSER_H_
#define PCANALYSER_H_

/*#include <message_filters/time_synchronizer.h>
#include <pcl_ros/pcl_nodelet.h>

#include <dynamic_reconfigure/server.h>
#include "pcl_ros/FeatureConfig.h"*/

#include <pcl_ros/pcl_nodelet.h>
#include <pcl/PointIndices.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/subscriber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl_visualizationpcl/visualizer.h/>
#include  <iostream>
#include <vector>

#include "my3dlaser/consts.h"
#include "my3dlaser/utils.h"
#include "my3dlaser/histogramhandler.h"

using namespace pcl_ros;

namespace myanalyeserns {

class PointCloudAnalyser : virtual public PCLNodelet<pcl::FPFHSignature33> {

private :

	//typedef PCLNodelet<pcl::FPFHSignature33> PCLNodeletBaseClass;
	//using PCLNodeletBaseClass::sync_input_indices_;

	typedef myanalyeserns::Histogram Histogram;
	typedef std::vector<Histogram> HistVector;

	typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> InputSynchronizer;

	std::vector<pcl::PointCloud<pcl::FPFHSignature33> > pcVector;

	typedef pcl::PointCloud<pcl::FPFHSignature33> MyPointCloud;

	int publisherCounter;

	/** \brief Synchronized input, and indices.*/
	boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2 > > sync_cloud_histogram;
	/** \brief The message filter subscriber for PointCloud2. */
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_filter_;
	/** \brief The message filter subscriber for FPFHSignature */
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_histogram_filter_;

	myanalyeserns::HistogramHandler histHandler;
	HistVector histVector;

	void listenerCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud);
	void cloud_histogram_callback(const sensor_msgs::PointCloud2::ConstPtr &voxelMsg, const sensor_msgs::PointCloud2::ConstPtr &histMsg);

	void testCompareObjects();
	void smallTests();
protected:



	ros::NodeHandle nh;

	/** \brief The surface PointCloud subscriber filter. */
	message_filters::Subscriber<sensor_msgs::PointCloud2> input_msg;

	ros::Subscriber subscriber;
	/** \brief The output PointCloud publisher. */
	ros::Publisher pub_output_;

	//! Synchronized input and normals
	boost::shared_ptr<InputSynchronizer> m_sync_input;

	float compareClouds(MyPointCloud input);

	/**
	 *	compare the given cloud histogram to all other histograms
	 */
	float compareClouds3Features(MyPointCloud input);

	void computeNormalizedHistogram(MyPointCloud cloud, float *f1, float *f2, float *f3);

public :

	ros::Publisher publishersVec[MAX_SINGLE_PUBLISHERS];

	virtual void onInit ();

};

#include "my3dlaser/pcanalyser.hpp"
}

#endif /* PCANALYSER_H_ */
