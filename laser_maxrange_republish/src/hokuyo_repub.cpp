/*
 * hokuyo_repub.cpp
 *
 *  Created on: Nov 13, 2012
 *      Author: golfcar
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "assert.h"

ros::Publisher *laser_pub_;
using namespace std;

sensor_msgs::LaserScan range_adjust(sensor_msgs::LaserScanConstPtr msg, double buffer)
{
    sensor_msgs::LaserScan ls = *msg;
    for (unsigned int i=0;i<ls.ranges.size();i++)
    {
        if(ls.ranges[i]>ls.range_max)
        {
            ls.ranges[i]=ls.range_max-buffer;
        }
        
        if(ls.ranges[i]<= 0.01)
        {
			  ls.ranges[i]=ls.range_max-buffer;
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
    return ls;
}

void scanCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    sensor_msgs::LaserScan ls = range_adjust(msg,0.05);
    laser_pub_->publish(ls);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hokuyo_repub");
  ros::NodeHandle n;
  ros::Publisher laser_pub= n.advertise<sensor_msgs::LaserScan>("hokuyo_filter",1);
  laser_pub_ = &laser_pub;
  ros::Subscriber sub = n.subscribe("hokuyo_scan", 1, scanCallback);
  ros::spin();

  return 0;
}
