/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>

#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <fmutil/StaticTransformPublisherDynReconfigConfig.h>


class TransformSender
{
public:
    //constructor
    TransformSender(double x, double y, double z,
                    double yaw, double pitch, double roll,
                    ros::Time time,
                    const std::string& frame_id, const std::string& child_frame_id)
    {
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );

        ros::NodeHandle nh("~");
        nh.setParam("tx", x);
        nh.setParam("ty", y);
        nh.setParam("tz", z);
        nh.setParam("roll", roll);
        nh.setParam("pitch", pitch);
        nh.setParam("yaw", yaw);
        nh.setParam("frame_id", frame_id);
        nh.setParam("child_frame_id", child_frame_id);
    }

    //Clean up ros connections
    ~TransformSender() { }

    void spin (ros::Duration sleeper)
    {
        // Define dynamic reconfigure callback, which gets called
        // immediately with level 0xffffffff.  The reconfig() method will
        // set initial parameter values, then open the device if it can.
        dynamic_reconfigure::Server<Config> srv;
        dynamic_reconfigure::Server<Config>::CallbackType f
                        = boost::bind(&TransformSender::reconfig, this, _1, _2);
        srv.setCallback(f);

        while( node_.ok() )
        {
            send(ros::Time::now() + sleeper);
            ROS_DEBUG("Sending transform from %s with parent %s\n",
                      transform_.frame_id_.c_str(), transform_.child_frame_id_.c_str());
            sleeper.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle node_;
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;

    // A function to call to send data periodically
    void send (ros::Time time)
    {
        transform_.stamp_ = time;
        broadcaster_.sendTransform(transform_);
    }

    /* dynamic parameter configuration */
    typedef fmutil::StaticTransformPublisherDynReconfigConfig Config;
    Config config_;

    /** Dynamic reconfigure callback
     *
     *  Called immediately when callback first defined. Called again
     *  when dynamic reconfigure starts or changes a parameter value.
     *
     *  @param newconfig new Config values
     *  @param level bit-wise OR of reconfiguration levels for all
     *               changed parameters (0xffffffff on initial call)
     **/
    void reconfig(Config &newconfig, uint32_t level)
    {
        ROS_DEBUG("dynamic reconfigure level 0x%x", level);

        tf::Quaternion q;
        q.setRPY(newconfig.roll, newconfig.pitch, newconfig.yaw);
        tf::Vector3 o(newconfig.tx, newconfig.ty, newconfig.tz);
        transform_.setData( tf::Transform(q, o) );
    }

};

int main(int argc, char ** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "static_transform_publisher_dyn_reconfig", ros::init_options::AnonymousName);

    if (argc == 10)
    {
        ros::Duration sleeper(atof(argv[9])/1000.0);

        if (strcmp(argv[7], argv[8]) == 0)
            ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[7], argv[8]);

        TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                                atof(argv[4]), atof(argv[5]), atof(argv[6]),
                                ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                                argv[7], argv[8]);

        tf_sender.spin(sleeper);

        return 0;

    }
    else
    {
        printf("A command line utility for manually sending a transform.\n");
        printf("It will periodicaly republish the given transform. \n");
        printf("Usage: static_transform_publisher_dyn_reconfig x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) \n");
        printf("OR \n");
        printf("Usage: static_transform_publisher_dyn_reconfig x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) \n");
        printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
        printf("of the child_frame_id.  \n");
        ROS_ERROR("static_transform_publisher_dyn_reconfig exited due to not having the right number of arguments");
        return -1;
    }


}

