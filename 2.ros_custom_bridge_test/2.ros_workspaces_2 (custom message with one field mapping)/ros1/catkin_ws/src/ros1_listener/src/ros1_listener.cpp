#include "ros1_listener.h"


/*******************************************************************************
 * @brief Constructor
 * @param[in] nh [ros::NodeHandle]
 * @param[in] nh_private [ros::NodeHandle]
 */
R1L::R1L(
  ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  subscriber_ = nh_.subscribe("chatter", 1, &R1L::callback, this);
}


/*******************************************************************************
 * @brief Destructor
 * @params void
 */
R1L::~R1L()
{
  ROS_INFO("[ros1_listener] Destroying ros1_listener");
}


/*******************************************************************************
 * @brief Callback
 * @param[in] msg [const std_msgs::String] The msg
 * @return void
 */
  void
R1L::callback(const std_msgs::String& msg)
{
  ROS_INFO("ROS1_LISTENER callback. Received %s", msg.data.c_str());
}
