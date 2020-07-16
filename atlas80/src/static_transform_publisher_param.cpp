#include <cstdio>
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle private_nh("~");
  ros::NodeHandle node;
  string parent_frame_id, child_frame_id;
  double roll, pitch, yaw, x, y, z, ms;
  private_nh.param("parent_frame_id", parent_frame_id, string("parent_frame"));
  private_nh.param("child_frame_id", child_frame_id, string("child_frame")); 
  private_nh.param("roll", roll, 0.0);
  private_nh.param("pitch", pitch, 0.0);
  private_nh.param("yaw", yaw, 0.0);
  private_nh.param("x", x, 0.0);
  private_nh.param("y", y, 0.0);
  private_nh.param("z", z, 0.0);
  private_nh.param("ms", ms, 10.0);
  tf::StampedTransform transform;
  tf::TransformBroadcaster broadcaster;
  tf::Quaternion q;
  q.setRPY(roll, pitch,yaw);
  ros::Duration sleeper(ms/1000.0);
  transform = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), ros::Time::now()+sleeper, parent_frame_id, child_frame_id );
  
  
  while(node.ok())
  {
    transform.stamp_ = ros::Time::now() + sleeper;
    broadcaster.sendTransform(transform);
    ROS_DEBUG("Sending transform from %s with parent %s\n", parent_frame_id.c_str(), child_frame_id.c_str());
    sleeper.sleep();
  }

  return 0;

};

