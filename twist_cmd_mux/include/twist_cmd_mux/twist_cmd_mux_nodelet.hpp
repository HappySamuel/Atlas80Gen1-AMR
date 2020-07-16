/**
 * @file /include/twist_cmd_mux/twist_cmd_mux_nodelet.hpp
 *
 * @brief Structure for the twist_cmd_mux.
 *
 * License: BSD
 *   https://github.com/mit-racecar/racecar/blob/master/ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef AGV_TWIST_CMD_MUX_HPP_
#define AGV_TWIST_CMD_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include "twist_cmd_mux/reloadConfig.h"
#include "twist_cmd_mux/twist_cmd_subscribers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace twist_cmd_mux {

/*****************************************************************************
 ** TwistCmdMux
 *****************************************************************************/

class TwistCmdMuxNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

  TwistCmdMuxNodelet()
  {
    dynamic_reconfigure_server = NULL;
  }

  ~TwistCmdMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

private:
  TwistCmdSubscribers twist_cmd_sub; /**< Pool of twist_cmd topics subscribers */
  ros::Publisher mux_twist_cmd_pub; /**< Multiplexed twist command topic */
  ros::Publisher active_subscriber; /**< Currently allowed twist_cmd subscriber */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void twistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<twist_cmd_mux::reloadConfig> * dynamic_reconfigure_server;
  dynamic_reconfigure::Server<twist_cmd_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(twist_cmd_mux::reloadConfig &config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming twist command topic to bind it to twist_cmd callback
  class TwistCmdFunctor
  {
  private:
    unsigned int idx;
    TwistCmdMuxNodelet* node;

  public:
    TwistCmdFunctor(unsigned int idx, TwistCmdMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const geometry_msgs::Twist::ConstPtr& msg)
    {
      node->twistCmdCallback(msg, idx);
    }
  };

  // Functor assigned to each twist command messages source to bind it to timer callback
  class TimerFunctor
  {
  private:
    unsigned int idx;
    TwistCmdMuxNodelet* node;

  public:
    TimerFunctor(unsigned int idx, TwistCmdMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

} // namespace twist_cmd_mux

#endif /* AGV_TWIST_CMD_MUX_HPP_ */
