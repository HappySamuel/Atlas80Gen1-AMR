/*   Author : Samuel Chieng Kien Ho   */

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher *scan_pub_;
double ang_min, ang_max;
bool intensity;

inline double to_radians(double degrees) {
    return degrees * (M_PI / 180.0);
}

// "Crop" the Field of View (FOV) of the LaserScan by appointing both minimum and maximum angle
sensor_msgs::LaserScan reduce_fov(sensor_msgs::LaserScanConstPtr msg, double ang_min, double ang_max, bool allow_intensity){
    sensor_msgs::LaserScan ls = *msg;

    for(int i=0; i<msg->ranges.size(); i++){
        double ang = msg->angle_min + msg->angle_increment*i;
        if(ang < to_radians(ang_min)){
            ls.ranges.erase(ls.ranges.begin());
            ls.intensities.erase(ls.intensities.begin());
        }else if(ang > to_radians(ang_max)){
            ls.ranges.erase(ls.ranges.end()-1);
            ls.intensities.erase(ls.intensities.end()-1);
        }
    }
    if(allow_intensity == false){
        ls.intensities.resize(0);
    }
    ls.angle_min = to_radians(ang_min);
    ls.angle_max = to_radians(ang_max);

    return ls;
}

void scanCB(const sensor_msgs::LaserScanConstPtr msg){
    sensor_msgs::LaserScan ls = reduce_fov(msg, ang_min, ang_max, intensity);
    scan_pub_->publish(ls);
}



int main(int argc, char **argv){
    ros::init(argc, argv, "laser_fov_narrower");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    priv_nh.param("ang_min", ang_min, -181.0);
    priv_nh.param("ang_max", ang_max, 180.0);
    priv_nh.param("intensity", intensity);

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_filter", 1);
    scan_pub_ = &scan_pub;
    ros::Subscriber scan_sub = nh.subscribe("scan", 1, scanCB);

    ros::spin();
    return 0;
}
