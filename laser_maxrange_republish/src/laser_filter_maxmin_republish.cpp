/*
 * laser_maxrange_republish.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: golfcar
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "assert.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud_conversion.h>
//create a publisher pointer
ros::Publisher *laser_scan_pub_;

using namespace std;

//get those reading that's larger than the specified range and
//put it under the specified maximum range by the given buffer value


	double max_height_, min_height_;
	laser_geometry::LaserProjection projector_;
	string target_frame_ = "golfcart/base_link";
	tf::TransformListener *tf_;

	void scanCallback(sensor_msgs::LaserScanConstPtr scan);

	void scanCallback(sensor_msgs::LaserScanConstPtr scan)
	{
		sensor_msgs::LaserScan ls = *scan;
		sensor_msgs::PointCloud laser_cloud;
		sensor_msgs::PointCloud laser_cloud_filtered;

		try
		{
			//transform laser_scan data to point_cloud
			projector_.transformLaserScanToPointCloud(target_frame_, ls, laser_cloud, *tf_);
     
			//initialize point_cloud_filtered object
			laser_cloud_filtered.header.frame_id = target_frame_;
			laser_cloud_filtered.header.stamp = ros::Time::now();
	  
			//filter point_cloud to limit the height
			for(size_t i=0; i<ls.ranges.size();i++)
			{
				if(laser_cloud.points[i].z>=min_height_ && laser_cloud.points[i].z<=max_height_)
				{
					laser_cloud_filtered.points.push_back(laser_cloud.points[i]);
				}
				sensor_msgs::PointCloud2 laser_cloud_fitered_pc2;
				sensor_msgs::convertPointCloudToPointCloud2(laser_cloud_filtered, laser_cloud_fitered_pc2);
				//publish filtered point_cloud data
				laser_scan_pub_->publish(laser_cloud_fitered_pc2);
			}
		}

		catch (tf::TransformException& e)
		{
			ROS_ERROR("%s",e.what());
		}
	}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_max_min_height");

  ros::NodeHandle n;
  
  ros::NodeHandle priv_nh("~");
  priv_nh.param("max_height", max_height_, 6.4);
  priv_nh.param("min_height", min_height_, 2.0);

  tf_ = new tf::TransformListener();

  //define the publisher
  ros::Publisher laser_scan_pub= n.advertise<sensor_msgs::PointCloud2>("scan_filter",1);
  laser_scan_pub_ = &laser_scan_pub;
  
  //create the subscriber
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  laser_scan_sub_.subscribe(n, "scan_in", 10);
  
  //filter laser_scan data
  tf::MessageFilter<sensor_msgs::LaserScan> *laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);

  //goto scanCallback function and treat laser_scan_filter as input parameter
  laser_scan_filter_->registerCallback(boost::bind(&scanCallback, _1));

  ros::spin();

  return 0;
}
