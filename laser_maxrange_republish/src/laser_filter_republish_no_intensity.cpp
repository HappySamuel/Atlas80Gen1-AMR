/*
 * laser_maxrange_republish.cpp
 *
 *  Created on: Feb 7, 2018
 *      Author: Samuel Chieng Kien Ho
 *      Refer to: Golfcar
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "assert.h"
ros::Publisher *laser_pub_;
using namespace std;
//get those reading that's larger than the specified range and
//put it under the specified maximum range by the given buffer value
sensor_msgs::LaserScan range_max(sensor_msgs::LaserScanConstPtr msg, double buffer)
{
    sensor_msgs::LaserScan ls = *msg;
    for (unsigned int i=0;i<ls.ranges.size();i++)
    {
        if(ls.ranges[i]>ls.range_max)
        {
            ls.ranges[i]=ls.range_max-buffer;
        }
        else
        {
            ls.ranges[i]=ls.range_max+buffer;
        }
    }
    return ls;
}

//"crop" the field of view of the laser scan by the specified points
//equally from left and right most of the data stream
sensor_msgs::LaserScan reduce_fov(sensor_msgs::LaserScanConstPtr msg, int points)
{
    sensor_msgs::LaserScan ls = *msg;
    double inc = fabs(msg->angle_increment * points);

    ls.angle_min>0 ? ls.angle_min -= inc : ls.angle_min += inc;
    ls.angle_max>0 ? ls.angle_max -= inc : ls.angle_max += inc;

    assert(points<round(ls.ranges.size()/2.0));

    for(int i=0; i<points;i++)
    {
        ls.ranges.erase(ls.ranges.begin());
        ls.ranges.erase(ls.ranges.end()-1);
    }
    ls.intensities.resize(0); //SAMUEL - added - to remove intensities
    return ls;
}

int reduce_pts_;
void scanCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    sensor_msgs::LaserScan ls = reduce_fov(msg,reduce_pts_);
    laser_pub_->publish(ls);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  priv_nh.param("fov_truncate_pts", reduce_pts_, 80);
  ros::Publisher laser_pub= n.advertise<sensor_msgs::LaserScan>("scan_filter",1);
  laser_pub_ = &laser_pub;
  ros::Subscriber sub = n.subscribe("scan", 1, scanCallback);
  ros::spin();

  return 0;
}
