/*
 * pcanalyser.hpp
 *
 *  Created on: 17.08.2010
 *      Author: ropra
 */

void myanalyeserns::PointCloudAnalyser::onInit ()
{
	// Call the super onInit ()
	  PCLNodelet<pcl::PointXYZ>::onInit ();

	  nh = getMTPrivateNodeHandle();
	  //ros::NodeHandle nh;

	  pub_output_ = pnh_->template advertise<sensor_msgs::PointCloud2> ("output", 1);



	  // If surface is enabled, subscribe to the surface, connect the input-indices-surface trio and register
	  //sub_input_.subscribe (nh, "input", max_queue_size_,  bind (&PointCloudAnalyser<pcl::PointXYZ>::listenerCallback, this, _1));
	  //message_filters::Subscriber<sensor_msgs::PointCloud2> m_sub_input_filter(nh, "input", 1);
	  //m_sub_input_filter.registerCallback(bind (&PointCloudAnalyser::listenerCallback, this, _1));

	  pclros_filter.subscribe (nh, "input", 1,  bind (&PointCloudAnalyser::listenerCallback, this, _1));

}

void myanalyeserns::PointCloudAnalyser::listenerCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud) {

	NODELET_INFO("MESSAGE RECEIVED");
	pub_output_.publish (boost::make_shared <const sensor_msgs::PointCloud2> (cloud));
}
