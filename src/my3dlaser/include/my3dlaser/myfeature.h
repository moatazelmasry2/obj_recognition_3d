/*
 * MyFeature.h
 *
 *  Created on: 16.08.2010
 *      Author: ropra
 */

#ifndef MYFEATURE_H_
#define MYFEATURE_H_

#include <pcl/pcl_base.h>

#include <pcl/features/feature.h>
#include <pcl/PointIndices.h>
// PCL includes

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/ros/conversions.h>

#include "my3dlaser/subscriber.h"
#include "my3dlaser/publisher.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace myanalyeserns {

template <typename PointInT, typename PointOutT>
class MyFeature : virtual public pcl::Feature<PointInT, PointOutT> {

public :

	typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;
	typedef pcl::ModelCoefficients::ConstPtr ModelCoefficientsConstPtr;

	typedef pcl::Feature<PointInT, PointOutT> FeatureBaseClass;
	typedef typename pcl::KdTree<PointInT> KdTree;
	typedef typename pcl::KdTree<PointInT>::Ptr KdTreePtr;

	// Members derived from the base class
	/** \brief The ROS NodeHandle used for parameters, publish/subscribe, etc. */
	boost::shared_ptr<ros::NodeHandle> pnh_;
	/** \brief The maximum queue size (default: 1). */
	int max_queue_size_;
	/** \brief The message filter subscriber for PointCloud2. */
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_filter_;
	/** \brief The message filter subscriber for PointIndices. */
	message_filters::Subscriber<pcl::PointIndices> sub_indices_filter_;
	/** \brief Synchronized input, and indices.*/
	boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, pcl::PointIndices> > sync_input_indices_;

	using FeatureBaseClass::input_;
	using FeatureBaseClass::indices_;
	using FeatureBaseClass::surface_;
	using FeatureBaseClass::tree_;
	using FeatureBaseClass::k_;
	using FeatureBaseClass::search_radius_;
	using FeatureBaseClass::use_indices_;

	typedef pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;
	typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

	typedef pcl::PointCloud<PointOutT> PointCloudOut;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ROS nodelet attributes
	/** \brief The input PointCloud subscriber. */
	point_cloud::Subscriber<PointInT> sub_input_;

	/** \brief The output PointCloud publisher. */
	ros::Publisher pub_output_;

	/** \brief The surface PointCloud subscriber filter. */
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_surface_filter_;

	/** \brief Set to true if the nodelet needs to listen for incoming point clouds representing the search surface. */
	bool use_surface_;

	/** \brief Parameter for the spatial locator tree. By convention, the values represent:
	* 0: ANN (Approximate Nearest Neigbor library) kd-tree
	* 1: FLANN (Fast Library for Approximate Nearest Neighbors) kd-tree
	* 2: Organized spatial dataset index
	*/
	int spatial_locator_type_;

	boost::shared_ptr <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, pcl::PointIndices> > sync_input_surface_indices_;

	/** \brief Synchronized input, and surface.*/
	boost::shared_ptr <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > sync_input_surface_;

public :
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Test whether a given PointIndices message is "valid" (i.e., has values).
	* \param indices the point indices message to test
	* \param topic_name an optional topic name (only used for printing, defaults to "indices")
	*/
	inline bool
	isValid (const PointIndicesConstPtr &indices, const std::string &topic_name = "indices")
	{
	if (indices->indices.size () == 0)
	{
	  ROS_WARN ("Empty indices (values = %d) with stamp %f, and frame %s on topic %s received!", (int)indices->indices.size (), indices->header.stamp.toSec (), indices->header.frame_id.c_str (), pnh_->resolveName (topic_name).c_str ());
	  return (false);
	}
	return (true);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Test whether a given ModelCoefficients message is "valid" (i.e., has values).
	* \param indices the point indices message to test
	* \param topic_name an optional topic name (only used for printing, defaults to "model")
	*/
	inline bool
	isValid (const ModelCoefficientsConstPtr &model, const std::string &topic_name = "model")
	{
	if (model->values.size () == 0)
	{
	  ROS_WARN ("Empty model (values = %d) with stamp %f, and frame %s on topic %s received!", (int)model->values.size (), model->header.stamp.toSec (), model->header.frame_id.c_str (), pnh_->resolveName (topic_name).c_str ());
	  return (false);
	}
	return (true);
	}

	// Need to make this public, as nodelet::Nodelet::getName is protected
	inline const std::string& getName () const { return ("getName for MyFeature"); }
public :

	void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);

	virtual void init();

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Compute the feature and publish it. Internal method. */
	void computeAndPublish ();


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
	* \param cloud the pointer to the input point cloud
	* \param cloud_surface the pointer to the surface point cloud
	* \param indices the pointer to the input point cloud indices
	*/
	void input_surface_indices_callback (const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &cloud_surface, const pcl::PointIndicesConstPtr &indices);

};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointInT, typename PointNT, typename PointOutT>
  class MyFeatureFromNormals : virtual public MyFeature<PointInT, PointOutT>, virtual public pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    typedef pcl::FeatureFromNormals<PointInT, PointNT, PointOutT> FeatureFromNormalsBaseClass;
    typedef MyFeature<PointInT, PointOutT> FeatureBaseClass;

    typedef typename MyFeature<PointInT, PointOutT>::PointCloudIn PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef typename pcl::PointCloud<PointNT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    public:
      using MyFeature<PointInT, PointOutT>::getName;

      typedef typename MyFeature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    protected:
      // Members derived from the base class
      using FeatureFromNormalsBaseClass::normals_;

      using FeatureBaseClass::pnh_;
      using FeatureBaseClass::isValid;
      using FeatureBaseClass::max_queue_size_;
      using FeatureBaseClass::sub_input_filter_;
      using FeatureBaseClass::sub_indices_filter_;
      using FeatureBaseClass::sync_input_indices_;
      using FeatureBaseClass::sub_surface_filter_;
      using FeatureBaseClass::input_;
      using FeatureBaseClass::indices_;
      using FeatureBaseClass::use_indices_;
      using FeatureBaseClass::surface_;
      using FeatureBaseClass::pub_output_;
      using FeatureBaseClass::max_queue_size_;
      using FeatureBaseClass::spatial_locator_type_;
      using FeatureBaseClass::use_surface_;
      using FeatureBaseClass::k_;
      using FeatureBaseClass::search_radius_;
      using FeatureBaseClass::computeAndPublish;

    private:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // ROS nodelet attributes
      /** \brief The normals PointCloud subscriber filter. */
      message_filters::Subscriber<sensor_msgs::PointCloud2> sub_normals_filter_;

      /** \brief Synchronized input, and normals.*/
      boost::shared_ptr <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > sync_input_normals_;

      /** \brief Synchronized input, normals, and indices.*/
      boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, pcl::PointIndices> > sync_input_normals_indices_;

      /** \brief Synchronized input, normals, and surface.*/
      boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > sync_input_normals_surface_;

      /** \brief Synchronized input, normals, surface, and point indices.*/
      boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, pcl::PointIndices> > sync_input_normals_surface_indices_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Nodelet initialization routine. */
      virtual void init ();

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
        * \param cloud the pointer to the input point cloud
        * \param cloud_normals the pointer to the input point cloud normals
        * \param cloud_surface the pointer to the surface point cloud
        * \param indices the pointer to the input point cloud indices
        */
      void input_normals_surface_indices_callback (const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &cloud_normals, const sensor_msgs::PointCloud2ConstPtr &cloud_surface, const pcl::PointIndicesConstPtr &indices);
  };
}

#include "my3dlaser/myfeature.hpp"
#endif /* MYFEATURE_H_ */
