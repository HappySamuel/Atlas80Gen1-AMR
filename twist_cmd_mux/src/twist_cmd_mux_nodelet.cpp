/**
 * @file /src/twist_cmd_mux_nodelet.cpp
 *
 * @brief  Implementation for the twist command multiplexer
 *
 * License: BSD
 *   https://github.com/mit-racecar/racecar/blob/master/ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>

#include "twist_cmd_mux/twist_cmd_mux_nodelet.hpp"
#include "twist_cmd_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace twist_cmd_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void TwistCmdMuxNodelet::twistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx)
{
	// Reset timer for this source
	twist_cmd_sub[idx].timer.stop();
	twist_cmd_sub[idx].timer.start();

	twist_cmd_sub[idx].active = true;   // obviously his source is sending commands, so active

	// Give permit to publish to this source if it's the only active or is
	// already allowed or has higher priority that the currently allowed
	if ((twist_cmd_sub.allowed == VACANT) ||
			(twist_cmd_sub.allowed == idx)    ||
			(twist_cmd_sub[idx].priority > twist_cmd_sub[twist_cmd_sub.allowed].priority))
	{
		if (twist_cmd_sub.allowed != idx)
		{
			twist_cmd_sub.allowed = idx;

			// Notify the world that a new twist_cmd source took the control
			std_msgs::StringPtr acv_msg(new std_msgs::String);
			acv_msg->data = twist_cmd_sub[idx].name;
		if (active_subscriber) {
				active_subscriber.publish(acv_msg);
			}
		}

	if (mux_twist_cmd_pub){
			mux_twist_cmd_pub.publish(msg);
		}
	}
}

void TwistCmdMuxNodelet::timerCallback(const ros::TimerEvent& event, unsigned int idx)
{
	if (twist_cmd_sub.allowed == idx)
	{
		// No twist_cmd messages timeout happened to currently active source, so...
		twist_cmd_sub.allowed = VACANT;

		// ...notify the world that nobody is publishing on twist_cmd; its vacant
		std_msgs::StringPtr acv_msg(new std_msgs::String);
		acv_msg->data = "idle";
		if (active_subscriber){
			active_subscriber.publish(acv_msg);
		}
	}

	twist_cmd_sub[idx].active = false;
}

void TwistCmdMuxNodelet::onInit()
{
	ros::NodeHandle &nh = this->getPrivateNodeHandle();

	/*********************
	** Dynamic Reconfigure
	**********************/
	dynamic_reconfigure_cb = boost::bind(&TwistCmdMuxNodelet::reloadConfiguration, this, _1, _2);
	dynamic_reconfigure_server = new dynamic_reconfigure::Server<twist_cmd_mux::reloadConfig>(nh);
	dynamic_reconfigure_server->setCallback(dynamic_reconfigure_cb);

	active_subscriber = nh.advertise <std_msgs::String> ("active", 1, true); // latched topic

	// Notify the world that by now nobody is publishing on twist_cmd yet
	std_msgs::StringPtr active_msg(new std_msgs::String);
	active_msg->data = "idle";
	if (active_subscriber) {
		active_subscriber.publish(active_msg);
	}

	// could use a call to reloadConfiguration here, but it seems to automatically call it once with defaults anyway.
	NODELET_DEBUG("TwistCmdMux : successfully initialised");
}

void TwistCmdMuxNodelet::reloadConfiguration(twist_cmd_mux::reloadConfig &config, uint32_t unused_level)
{
	std::string yaml_cfg_file;
	ros::NodeHandle &nh = this->getNodeHandle();
	ros::NodeHandle &nh_priv = this->getPrivateNodeHandle();
	if( config.yaml_cfg_file == "" )
	{
		// typically fired on startup, so look for a parameter to set a default
		nh_priv.getParam("yaml_cfg_file", yaml_cfg_file);
	}
	else
	{
		yaml_cfg_file = config.yaml_cfg_file;
	}

	/*********************
	** Yaml File Parsing
	**********************/
	std::ifstream ifs(yaml_cfg_file.c_str(), std::ifstream::in);
	if (ifs.good() == false)
	{
		NODELET_ERROR_STREAM("TwistCmdMux : configuration file not found [" << yaml_cfg_file << "]");
		return;
	}
	// probably need to bring the try catches back here
	YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
	doc = YAML::Load(ifs);
#else
	YAML::Parser parser(ifs);
	parser.GetNextDocument(doc);
#endif

	/*********************
	** Output Publisher
	**********************/
	std::string output_name("output");
#ifdef HAVE_NEW_YAMLCPP
	if ( doc["publisher"] ) {
		doc["publisher"] >> output_name;
	}
#else
	const YAML::Node *node = doc.FindValue("publisher");
	if ( node != NULL ) {
		*node >> output_name;
	}
#endif
	mux_twist_cmd_pub = nh_priv.advertise <geometry_msgs::Twist> (output_name, 10);

	/*********************
	** Input Subscribers
	**********************/
	try {
		twist_cmd_sub.configure(doc["subscribers"]);
	}
	catch(EmptyCfgException& e) {
		NODELET_WARN("TwsitCmdMux : yaml configured zero subscribers, check yaml content.");
	}
	catch(YamlException& e) {
		NODELET_ERROR_STREAM("TwistCmdMux : yaml parsing problem [" << std::string(e.what()) + "]");
	}

	// Publishers and subscribers
	for (unsigned int i = 0; i < twist_cmd_sub.size(); i++)
	{
		twist_cmd_sub[i].subs =
				nh_priv.subscribe<geometry_msgs::Twist>(twist_cmd_sub[i].topic, 10, TwistCmdFunctor(i, this));

		// Create (stopped by now) a one-shot timer for every subscriber
		twist_cmd_sub[i].timer =
				nh_priv.createTimer(ros::Duration(twist_cmd_sub[i].timeout), TimerFunctor(i, this), true, false);

		NODELET_DEBUG("TwistCmdMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
							twist_cmd_sub[i].name.c_str(), twist_cmd_sub[i].topic.c_str(),
							twist_cmd_sub[i].priority, twist_cmd_sub[i].timeout);
	}

	NODELET_INFO_STREAM("TwistCmdMux : (re)configured [" << yaml_cfg_file << "]");
	ifs.close();
}

} // namespace twist_cmd_mux

PLUGINLIB_EXPORT_CLASS(twist_cmd_mux::TwistCmdMuxNodelet, nodelet::Nodelet);
