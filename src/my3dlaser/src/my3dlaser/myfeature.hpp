/*
 * mz3danalyser.hpp
 *
 *  Created on: 16.08.2010
 *      Author: ropra
 */

template <typename PointInT, typename PointOutT>
void myanalyeserns::MyFeature<PointInT, PointOutT>::init() {

	//ros::init(argc, argv, "MyFeature");
	ros::NodeHandle n;


	//ros::spin();

	pub_output_ = pnh_->template advertise<sensor_msgs::PointCloud2> ("output", max_queue_size_);
	k_ = 20;
	search_radius_=0.0;
	spatial_locator_type_=0;

	sync_input_indices_         = boost::make_shared <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, pcl::PointIndices> >(3);
	sync_input_surface_         = boost::make_shared <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> >(3);
	sync_input_surface_indices_ = boost::make_shared <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, pcl::PointIndices> >(3);

	use_surface_=true;
	use_indices_=true;

	sub_input_filter_.subscribe (*pnh_, "input", max_queue_size_);
	// If indices are enabled, subscribe to the indices
	sub_indices_filter_.subscribe (*pnh_, "indices", max_queue_size_);

	// If surface is enabled, subscribe to the surface, connect the input-indices-surface trio and register
	sub_surface_filter_.subscribe (*pnh_, "surface", max_queue_size_);
	sync_input_surface_indices_->connectInput (sub_input_filter_, sub_surface_filter_, sub_indices_filter_);
	sync_input_surface_indices_->registerCallback (bind (&MyFeature<PointInT, PointOutT>::input_surface_indices_callback, this, _1, _2, _3));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
  * \param cloud the pointer to the input point cloud
  * \param cloud_surface the pointer to the surface point cloud
  * \param indices the pointer to the input point cloud indices
  */
template <typename PointInT, typename PointOutT> void
  myanalyeserns::MyFeature<PointInT, PointOutT>::input_surface_indices_callback (
      const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &cloud_surface, const pcl::PointIndicesConstPtr &indices)
{
  if (!isValid (cloud) || !isValid (cloud_surface, "surface") || !isValid (indices))
    return;

  ROS_DEBUG ("[input_surface_indices_callback]\n"
                 "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                         - PointIndices with %d values, stamp %f, and frame %s on topic %s received.",
             cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), cloud->header.stamp.toSec (), cloud->header.frame_id.c_str (), pnh_->resolveName ("input").c_str (),
             cloud_surface->width * cloud_surface->height, pcl::getFieldsList (*cloud_surface).c_str (), cloud_surface->header.stamp.toSec (), cloud_surface->header.frame_id.c_str (), pnh_->resolveName ("surface").c_str (),
             (int)indices->indices.size (), indices->header.stamp.toSec (), indices->header.frame_id.c_str (), pnh_->resolveName ("indices").c_str ());

  if ((int)(cloud->width * cloud->height) < k_)
  {
    ROS_ERROR ("[input_surface_indices_callback] Requested number of k-nearest neighbors (%d) is larger than the PointCloud size (%d)!", k_, (int)(cloud->width * cloud->height));
    return;
  }

  PointCloudIn input;
  pcl::fromROSMsg (*cloud, input);                  // input_ = cloud;
  input_ = boost::make_shared <const PointCloudIn> (input);

  PointCloudIn surface;
  pcl::fromROSMsg (*cloud_surface, surface);        // surface_ = cloud_surf;
  surface_ = boost::make_shared <const PointCloudIn> (surface);

  indices_ = boost::make_shared <const std::vector<int> > (indices->indices);
  computeAndPublish ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Compute the feature and publish it. Internal method. */
template <typename PointInT, typename PointOutT> void
  myanalyeserns::MyFeature<PointInT, PointOutT>::computeAndPublish ()
{
  // Initialize the spatial locator
  initTree (spatial_locator_type_, tree_, k_);

  // Estimate the feature
  PointCloudOut output;
  compute (output);
  if (output.points.size () == 0)
  {
    ROS_WARN ("[computeAndPublish] Output PointCloud has no data points on topic %s (parameters might not be set correctly)!", pnh_->resolveName ("output").c_str ());
    return;
  }

  // Convert from the templated output to the PointCloud blob
  sensor_msgs::PointCloud2 output_blob;
  pcl::toROSMsg (output, output_blob);
  // Publish a Boost shared ptr const data
  pub_output_.publish (boost::make_shared <const sensor_msgs::PointCloud2> (output_blob));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////	Implementation of FeatureFromNormal  /////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief ROS initialization routine. */
template <typename PointInT, typename PointNT, typename PointOutT> void
  myanalyeserns::MyFeatureFromNormals<PointInT, PointNT, PointOutT>::init()
{
	pub_output_ = pnh_->template advertise<sensor_msgs::PointCloud2> ("output", max_queue_size_);

	sync_input_normals_ = boost::make_shared <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > (3);

	//Define default configs
	k_ = 20;
	search_radius_= 0.0;
	spatial_locator_type_= 0;
	use_surface_ = true;
	use_indices_ = true;
	use_surface_ = true;

	sub_input_filter_.subscribe (*pnh_, "input", max_queue_size_);
	sub_normals_filter_.subscribe (*pnh_, "normals", max_queue_size_);
	sync_input_normals_->connectInput (sub_input_filter_, sub_normals_filter_);

	// Creating subscribers and connecting them
	// registering a callback
	sub_indices_filter_.subscribe (*pnh_, "indices", max_queue_size_);
	// If surface is enabled, subscribe to the surface, connect the input-indices-surface trio and register
	sub_surface_filter_.subscribe (*pnh_, "surface", max_queue_size_);
	sync_input_normals_surface_indices_ = boost::make_shared <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, pcl::PointIndices> > (3);
	sync_input_normals_surface_indices_->connectInput (sub_input_filter_, sub_normals_filter_, sub_surface_filter_, sub_indices_filter_);
	sync_input_normals_surface_indices_->registerCallback (bind (&MyFeatureFromNormals<PointInT, PointNT, PointOutT>::input_normals_surface_indices_callback, this, _1, _2, _3, _4));

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
  * \param cloud the pointer to the input point cloud
  * \param cloud_normals the pointer to the input point cloud normals
  * \param cloud_surface the pointer to the surface point cloud
  * \param indices the pointer to the input point cloud indices
  */
template <typename PointInT, typename PointNT, typename PointOutT> void
  myanalyeserns::MyFeatureFromNormals<PointInT, PointNT, PointOutT>::input_normals_surface_indices_callback (
      const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &cloud_normals,
      const sensor_msgs::PointCloud2ConstPtr &cloud_surface, const pcl::PointIndicesConstPtr &indices)
{
  if (!isValid (cloud) || !isValid (cloud_normals, "normals") || !isValid (cloud_surface, "surface") || !isValid (indices))
    return;

  ROS_DEBUG ("[input_normals_surface_indices_callback]\n"
                 "                                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                                 - PointIndices with %d values, stamp %f, and frame %s on topic %s received.",
                 cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), cloud->header.stamp.toSec (), cloud->header.frame_id.c_str (), pnh_->resolveName ("input").c_str (),
                 cloud_surface->width * cloud_surface->height, pcl::getFieldsList (*cloud_surface).c_str (), cloud_surface->header.stamp.toSec (), cloud_surface->header.frame_id.c_str (), pnh_->resolveName ("surface").c_str (),
                 cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), cloud_normals->header.stamp.toSec (), cloud_normals->header.frame_id.c_str (), pnh_->resolveName ("normals").c_str (),
                 (int)indices->indices.size (), indices->header.stamp.toSec (), indices->header.frame_id.c_str (), pnh_->resolveName ("indices").c_str ());

  if ((int)(cloud->width * cloud->height) < k_)
  {
    ROS_ERROR ("[input_normals_surface_indices_callback] Requested number of k-nearest neighbors (%d) is larger than the PointCloud size (%d)!", k_, (int)(cloud->width * cloud->height));
    return;
  }
  PointCloudIn input;
  pcl::fromROSMsg (*cloud, input);                // input_ = cloud;
  input_ = boost::make_shared <const PointCloudIn> (input);

  PointCloudN normals;
  pcl::fromROSMsg (*cloud_normals, normals);      // normals_ = cloud_normals;
  normals_ = boost::make_shared <const PointCloudN> (normals);

  PointCloudIn surface;
  pcl::fromROSMsg (*cloud_surface, surface);      // surface_ = cloud_surf;
  surface_ = boost::make_shared <const PointCloudIn> (surface);

  indices_ = boost::make_shared <const std::vector<int> > (indices->indices);

  computeAndPublish ();
}

