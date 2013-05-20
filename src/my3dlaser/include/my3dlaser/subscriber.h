/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: subscriber.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */
/**

\author Patrick Mihelich

@b Subscriber represents a ROS subscriber for the templated PointCloud implementation. 

**/

#ifndef PCL_ROS_SUBSCRIBER_H_
#define PCL_ROS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include "pcl/ros/conversions.h"

namespace point_cloud
{
  template <typename PointT>
  class Subscriber
  {
    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::function<void(const typename PointCloud::ConstPtr&)> Callback;
      
      Subscriber () {}

      Subscriber (ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                  const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr (),
                  const ros::TransportHints& transport_hints = ros::TransportHints ())
      {
        subscribe (nh, base_topic, queue_size, callback, tracked_object, transport_hints);
      }

      void 
        subscribe (ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                   const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr (),
                   const ros::TransportHints& transport_hints = ros::TransportHints ())
      {
        user_cb_ = callback;
        sub_ = nh.subscribe (base_topic, queue_size, &Subscriber::internalCB, this, transport_hints);
      }

      std::string
        getTopic ()
      {
        return (sub_.getTopic ());
      }

      void
        shutdown ()
      {
        sub_.shutdown ();
      }
     
    private:
      ros::Subscriber sub_;
      Callback user_cb_;
      pcl::MsgFieldMap field_map_;

      void
        internalCB (const sensor_msgs::PointCloud2ConstPtr& msg)
      {
        // On first message, figure out how to map serialized point data to PointT.
        /// @todo Currently assume all datatypes are the same (e.g. no float<->double conversions)
        if (field_map_.empty ())
          pcl::createMapping<PointT>(*msg, field_map_);

        // Convert to templated point cloud.
        boost::shared_ptr<PointCloud> cloud = boost::make_shared<PointCloud>();
        fromROSMsg (*msg, *cloud, field_map_);

        // Invoke user callback with nice point cloud type.
        user_cb_(cloud);
      }
  };
    
  template <>
  class Subscriber<sensor_msgs::PointCloud2>
  {
    public:
      typedef boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> Callback;
      
      Subscriber () { }

      Subscriber (ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                  const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr (),
                  const ros::TransportHints& transport_hints = ros::TransportHints ())
      {
        subscribe (nh, base_topic, queue_size, callback, tracked_object, transport_hints);
      }

      void 
        subscribe (ros::NodeHandle& nh, const std::string &base_topic, uint32_t queue_size,
                   const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr (),
                   const ros::TransportHints& transport_hints = ros::TransportHints ())
      {
        user_cb_ = callback;
        sub_ = nh.subscribe (base_topic, queue_size, &Subscriber::internalCB, this, transport_hints);
      }

      std::string
        getTopic ()
      {
        return (sub_.getTopic ());
      }

      void
        shutdown ()
      {
        sub_.shutdown ();
      }
      
    private:
      ros::Subscriber sub_;
      Callback user_cb_;

      void 
        internalCB (const sensor_msgs::PointCloud2ConstPtr& msg)
      {
        user_cb_ (msg);
      }
  };
}

#endif  //#ifndef PCL_ROS_SUBSCRIBER_H_
