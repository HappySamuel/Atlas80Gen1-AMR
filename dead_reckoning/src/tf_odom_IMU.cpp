/** Combines odometry (distance travelled) and IMU (yaw rate) to get position
 *  estimate.
 *
 * Gets the input from encoders and IMU ('imu/data', yaw rate only),
 * publishes the resulting pose estimate as an Odometry message on the 'odom' topic.
 */

#include <cmath>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fmutil/fm_math.h>

#include <dead_reckoning/Encoders.h>

#include "tf/transform_broadcaster.h"
#define PI 3.14159265359
class OdoIMU
{
    public:
        OdoIMU();

    private:
        void encodersCallBack(nav_msgs::Odometry);
        void imuCallBack(sensor_msgs::Imu);
        void reinitializeCallBack(geometry_msgs::PoseWithCovarianceStamped);
        void publishOdo();

        ros::NodeHandle nh_;
        ros::Subscriber enc_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber reinitialize_sub_;
        ros::Publisher odo_imu_pub_;

        std::string parent_frame_id_, child_frame_id_;
        geometry_msgs::Point position_, last_position_, now_position_;
        double linear_speed_, angular_speed_;
        bool initialized_, reinitialized_;
        double roll_, pitch_, yaw_, yaw_now_;
	    double roll_ref_, pitch_ref_, yaw_ref_;
	    double roll_init_, pitch_init_;
	    bool roll_pitch_initialized_;
        double dist_pre_;
        double yaw_pre_, yaw_drift_, yaw_minus_;

        diagnostic_updater::Updater diag_updater_;
        double diag_min_freq_, diag_max_freq_;
        diagnostic_updater::FrequencyStatusParam diag_param_fs_;
        diagnostic_updater::FrequencyStatus diag_task_fs_;


    	tf::TransformBroadcaster *tfb_;
};



OdoIMU::OdoIMU()
: parent_frame_id_("odom"), child_frame_id_("base_link"),
  diag_min_freq_(90), diag_max_freq_(110),
  diag_param_fs_(&diag_min_freq_, &diag_max_freq_),
  diag_task_fs_(diag_param_fs_)
{
    enc_sub_ = nh_.subscribe("motor/odom", 1, &OdoIMU::encodersCallBack, this);
    imu_sub_ = nh_.subscribe("imu/data_raw", 1, &OdoIMU::imuCallBack, this);
    reinitialize_sub_ = nh_.subscribe("/initialpose", 1, &OdoIMU::reinitializeCallBack, this);
    odo_imu_pub_ = nh_.advertise<nav_msgs::Odometry>("imu/odom", 1);

    ros::NodeHandle n("~");
    n.getParam("parent_frame_id", parent_frame_id_);
    n.getParam("child_frame_id", child_frame_id_);
    initialized_ = false;
    reinitialized_ = false;
    roll_pitch_initialized_ = false;
    yaw_drift_ = 0;
    roll_ = pitch_ = yaw_ = NAN;
    roll_ref_ = 0.0;
    pitch_ref_ = 0.0;
    yaw_ref_ = 0.0;
	tfb_ = new tf::TransformBroadcaster();
    diag_updater_.setHardwareID("none");
    diag_updater_.add(diag_task_fs_);
}

#define R2D(a) ( (int)(fmutil::r2d( fmutil::angModPI(a) )) )
using namespace std;
void OdoIMU::imuCallBack(sensor_msgs::Imu imuMsg)
{
  
    // Get RPY from the IMU
    tf::Quaternion qt;
    tf::quaternionMsgToTF(imuMsg.orientation, qt);
    tf::Matrix3x3(qt).getRPY(roll_, pitch_, yaw_);
    yaw_+=PI/2;
    if(!roll_pitch_initialized_)
    {
      roll_pitch_initialized_ = true;
      roll_init_ = roll_;
      pitch_init_ = pitch_;
    }
    ROS_DEBUG("rpy: %d, %d, %d (degrees)", R2D(roll_), R2D(pitch_), R2D(yaw_));
    double p = imuMsg.angular_velocity.x;
    double q = imuMsg.angular_velocity.y;
    double r = imuMsg.angular_velocity.z;
    double roll_ref_dot = p + (q*sin(roll_ref_)+r*cos(roll_ref_))*tan(pitch_ref_);
    double pitch_ref_dot = q*cos(roll_ref_)-r*sin(roll_ref_);
    //surprise surprise, that's the exact answer to IMU's built in orientation output!!!
    double yaw_ref_dot = (r*cos(roll_ref_)+q*sin(roll_ref_))/cos(pitch_ref_dot);
    roll_ref_+=(roll_ref_dot-0.05*(roll_ref_-(roll_-roll_init_)))*0.01;
    pitch_ref_+=(pitch_ref_dot-0.05*(pitch_ref_-(pitch_-pitch_init_)))*0.01;
    yaw_ref_+=yaw_ref_dot*0.01;
    //simple bias estimate    
    cout<<fmutil::r2d(roll_ref_)<<" "<<fmutil::r2d(roll_)<<" ";
    cout<<fmutil::r2d(pitch_ref_)<<" "<<fmutil::r2d(pitch_)<<" ";
    cout<<fmutil::r2d(yaw_ref_)<<" "<<fmutil::r2d(yaw_)<<" ";
    cout<<p<<" "<<q<<" "<<r<<endl;
}


void OdoIMU::encodersCallBack(nav_msgs::Odometry encMsg)
{
    //handle the nan issue. This occurs when the imu is restarted
    if( isnan(roll_) || isnan(pitch_) || isnan(yaw_) )
        return;
	yaw_now_ = yaw_;
    if( ! initialized_ )
    {
        yaw_pre_ = yaw_now_;
        yaw_minus_ = yaw_now_;
        initialized_ = true;
        return;
    }
    if( reinitialized_ )
    {
        position_.x = 0;
        position_.y = 0;
        position_.z = 0;
        reinitialized_ = false;
    }
    //Ensure that the orientation always start from yaw=0.
    //That's the assumption made for odometry calculation

    // Only integrate yaw when the car is moving
    if( fabs(encMsg.twist.twist.linear.x) < 0.005 and fabs(encMsg.twist.twist.angular.z) < 0.001) //the car is not moving
        yaw_drift_ += yaw_now_ - yaw_minus_;
    yaw_minus_ = yaw_now_;
    yaw_now_ -= yaw_pre_ + yaw_drift_;
    //std::cout <<yaw_ <<' ' <<yaw_drift_ <<std::endl;
    now_position_ = encMsg.pose.pose.position;

    double r11 = cos(yaw_now_)*cos(pitch_ref_);
    double r21 = -sin(yaw_now_)*cos(pitch_ref_);
    double r31 = sin(pitch_ref_);
    double d_dist = sqrt(pow((now_position_.x-last_position_.x),2) + pow((now_position_.y-last_position_.y),2) + pow((now_position_.z-last_position_.z),2));
    position_.x += d_dist * r11;
    position_.y += d_dist * r21;
    position_.z = 0.0;

    linear_speed_ = encMsg.twist.twist.linear.x;
    angular_speed_ = encMsg.twist.twist.angular.z;

    ROS_DEBUG("Pose: x=%.2f, y=%.2f, th=%ddeg", position_.x, position_.y, R2D(yaw_now_));
    publishOdo();
    diag_task_fs_.tick();
    diag_updater_.update();
    last_position_ = encMsg.pose.pose.position;
}

void OdoIMU::reinitializeCallBack(geometry_msgs::PoseWithCovarianceStamped reinit_msg)
{
    reinitialized_ = true;
}

void OdoIMU::publishOdo()
{
    // Create the Odometry msg
    nav_msgs::Odometry odoImuMsg;
    odoImuMsg.header.stamp = ros::Time::now();
    odoImuMsg.header.frame_id = parent_frame_id_;
    odoImuMsg.child_frame_id = child_frame_id_;
    odoImuMsg.pose.pose.position = position_;
    odoImuMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -yaw_now_);
    odoImuMsg.twist.twist.linear.x = linear_speed_;
    odoImuMsg.twist.twist.angular.z = angular_speed_;
    // Publish it
    odo_imu_pub_.publish(odoImuMsg);


//    tf::StampedTransform tmp_tf_stamped(tf::Transform(tf::Quaternion(odoImuMsg.pose.pose.orientation.x, odoImuMsg.pose.pose.orientation.y, odoImuMsg.pose.pose.orientation.z, odoImuMsg.pose.pose.orientation.w), tf::Point(odoImuMsg.pose.pose.position.x, odoImuMsg.pose.pose.position.y, odoImuMsg.pose.pose.position.z)), odoImuMsg.header.stamp, parent_frame_id_, child_frame_id_);
//    tfb_->sendTransform(tmp_tf_stamped);

}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_odom_Imu");
    OdoIMU odomimu;
    ros::spin();
    return 0;
}
