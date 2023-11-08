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
  ROS_INFO("[ros1_out] Constructor");

  service_server_ = nh_.advertiseService("service_1_out",
      &R1L::serviceServerCallback, this);

  service_client_1_out_to_1_in_ =
    nh_.serviceClient<std_srvs::Trigger>("service_1_in");
  service_client_1_out_to_2_in_ =
    nh_.serviceClient<std_srvs::Trigger>("service_2_in");
  service_client_1_out_to_2_out_ =
    nh_.serviceClient<std_srvs::Trigger>("service_2_out");

  timer_1_out_to_1_in_  = nh_.createTimer(
    ros::Duration(1.0), &R1L::timer_1_out_to_1_in_callback,  this);
  timer_1_out_to_2_in_  = nh_.createTimer(
    ros::Duration(1.0), &R1L::timer_1_out_to_2_in_callback,  this);
  timer_1_out_to_2_out_ = nh_.createTimer(
    ros::Duration(1.0), &R1L::timer_1_out_to_2_out_callback, this);
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
bool R1L::serviceServerCallback(
  std_srvs::Trigger::Request& req,
  std_srvs::Trigger::Response& res)
{
  ROS_INFO("[ros1_out] Service called");
  res.success = 1;
  res.message = "ros1_out_success_message";
  return true;
}


/*******************************************************************************
*/
void R1L::timer_1_out_to_1_in_callback(const ros::TimerEvent& e)
{
  std_srvs::Trigger t;

  ROS_INFO("[ros1_out] Requesting service_1_in");
  if(service_client_1_out_to_1_in_.call(t))
    ROS_INFO("[ros1_out] Served service_1_in");
  else
    ROS_ERROR("[ros1_out] NOT Served service_1_in");
}
void R1L::timer_1_out_to_2_in_callback(const ros::TimerEvent& e)
{
  std_srvs::Trigger t;

  ROS_INFO("[ros1_out] Requesting service_2_in");
  if(service_client_1_out_to_2_in_.call(t))
    ROS_INFO("[ros1_out] Served service_2_in");
  else
    ROS_ERROR("[ros1_out] NOT Served service_2_in");
}
void R1L::timer_1_out_to_2_out_callback(const ros::TimerEvent& e)
{
  std_srvs::Trigger t;

  ROS_INFO("[ros1_out] Requesting service_2_out");
  if(service_client_1_out_to_2_out_.call(t))
    ROS_INFO("[ros1_out] Served service_2_out");
  else
    ROS_ERROR("[ros1_out] NOT Served service_2_out");
}
