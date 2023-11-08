#include "ros1_out.h"


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
  subscriber_1_in_to_1_out_  = nh_.subscribe("topic_1_in_to_1_out",  1, &R1L::callback_1_in_to_1_out,  this);
  subscriber_2_in_to_1_out_  = nh_.subscribe("topic_2_in_to_1_out",  1, &R1L::callback_2_in_to_1_out,  this);
  subscriber_2_out_to_1_out_ = nh_.subscribe("topic_2_out_to_1_out", 1, &R1L::callback_2_out_to_1_out, this);

  publisher_1_out_to_1_in_  = nh_.advertise<std_msgs::String>("topic_1_out_to_1_in",  1);
  publisher_1_out_to_2_in_  = nh_.advertise<std_msgs::String>("topic_1_out_to_2_in",  1);
  publisher_1_out_to_2_out_ = nh_.advertise<std_msgs::String>("topic_1_out_to_2_out", 1);

  timer_1_out_to_1_in_  = nh_.createTimer(ros::Duration(1.0), &R1L::timer_1_out_to_1_in_callback,  this);
  timer_1_out_to_2_in_  = nh_.createTimer(ros::Duration(1.0), &R1L::timer_1_out_to_2_in_callback,  this);
  timer_1_out_to_2_out_ = nh_.createTimer(ros::Duration(1.0), &R1L::timer_1_out_to_2_out_callback, this);
}


/*******************************************************************************
 * @brief Destructor
 * @params void
 */
R1L::~R1L()
{
  ROS_INFO("[ros1_out] Destroying ROS 1 out");
}




/*******************************************************************************
*/
void R1L::callback_1_in_to_1_out(const std_msgs::String& msg)
{
  ROS_INFO("[ros1_out] callback 1_in_to_1_out callback. Received %s", msg.data.c_str());
}

void R1L::callback_2_in_to_1_out(const std_msgs::String& msg)
{
  ROS_INFO("[ros1_out] callback 2_in_to_1_out callback. Received %s", msg.data.c_str());
}

void R1L::callback_2_out_to_1_out(const std_msgs::String& msg)
{
  ROS_INFO("[ros1_out] callback 2_out_to_1_out callback. Received %s", msg.data.c_str());
}

/*******************************************************************************
*/
void R1L::timer_1_out_to_1_in_callback(const ros::TimerEvent& e)
{
  std_msgs::String s;
  s.data = "1_out_to_1_in";
  publisher_1_out_to_1_in_.publish(s);

  ROS_INFO("[ros1_out] Publishing %s", s.data.c_str());
}

void R1L::timer_1_out_to_2_in_callback(const ros::TimerEvent& e)
{
  std_msgs::String s;
  s.data = "1_out_to_2_in";
  publisher_1_out_to_2_in_.publish(s);

  ROS_INFO("[ros1_out] Publishing %s", s.data.c_str());
}

void R1L::timer_1_out_to_2_out_callback(const ros::TimerEvent& e)
{
  std_msgs::String s;
  s.data = "1_out_to_2_out";
  publisher_1_out_to_2_out_.publish(s);

  ROS_INFO("[ros1_out] Publishing %s", s.data.c_str());
}
