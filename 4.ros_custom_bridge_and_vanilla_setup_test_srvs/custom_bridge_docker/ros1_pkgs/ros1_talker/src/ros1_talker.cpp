#include "ros1_talker.h"


/*******************************************************************************
 * @brief Constructor
 * @param[in] nh [ros::NodeHandle]
 * @param[in] nh_private [ros::NodeHandle]
 */
R1T::R1T(
  ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("[ros1_talker] Creating timers");

  // Service offered
  service_server_ = nh_.advertiseService("service_1_in",
      &R1T::serviceServerCallback, this);
  service_server_custom_ = nh_.advertiseService("service_1_in_custom",
      &R1T::serviceServerCustomCallback, this);


  // Timers for calling other nodes' services
  timer_1_in_to_2_in_  =
    nh_.createTimer(ros::Duration(1), &R1T::timer_1_in_to_1_out_callback, this);
  timer_1_in_to_2_out_ =
    nh_.createTimer(ros::Duration(1), &R1T::timer_1_in_to_2_out_callback, this);
  timer_1_in_to_1_out_ =
    nh_.createTimer(ros::Duration(1), &R1T::timer_1_in_to_2_in_callback,  this);

  // Services called
  service_client_1_in_to_2_in_ =
    nh_.serviceClient<std_srvs::Trigger>("service_2_in");
  service_client_1_in_to_2_out_ =
    nh_.serviceClient<std_srvs::Trigger>("service_2_out");
  service_client_1_in_to_1_out_ =
    nh_.serviceClient<std_srvs::Trigger>("service_1_out");
}


/*******************************************************************************
 * @brief Destructor
 * @params void
 */
R1T::~R1T()
{
  ROS_INFO("[ros1_talker] Destroying ros1_talker");
}


/*******************************************************************************
 * @brief Callback
 * @return void
 */
 bool
R1T::serviceServerCallback(
  std_srvs::Trigger::Request& req,
  std_srvs::Trigger::Response& res)
{
  ROS_INFO("[ros1_in] Service called");
  res.success = 1;
  res.message = "ros1_in_success_message";
  return true;
}


/*******************************************************************************
 * @brief Custom Callback
 * @return void
 */
  bool
R1T::serviceServerCustomCallback(
  ros1_talker::custom_srv::Request& req,
  ros1_talker::custom_srv::Response& res)
{
  ROS_INFO("[ros1_in] Custom service called");
  res.pose.header.stamp = ros::Time::now();
  return true;
}


/*******************************************************************************
*/
void R1T::timer_1_in_to_1_out_callback(const ros::TimerEvent& e)
{
  std_srvs::Trigger t;

  ROS_INFO("[ros1_in] Requesting service_1_out");
  if(service_client_1_in_to_1_out_.call(t))
    ROS_INFO("[ros1_in] Served service_1_out");
  else
    ROS_ERROR("[ros1_in] NOT Served service_1_out");
}
void R1T::timer_1_in_to_2_out_callback(const ros::TimerEvent& e)
{
  std_srvs::Trigger t;

  ROS_INFO("[ros1_in] Requesting service_2_out");
  if(service_client_1_in_to_2_out_.call(t))
    ROS_INFO("[ros1_in] Served service_2_out");
  else
    ROS_ERROR("[ros1_in] NOT Served service_2_out");
}
void R1T::timer_1_in_to_2_in_callback(const ros::TimerEvent& e)
{
  std_srvs::Trigger t;

  ROS_INFO("[ros1_in] Requesting service_2_in");
  if(service_client_1_in_to_2_in_.call(t))
    ROS_INFO("[ros1_in] Served service_2_in");
  else
    ROS_ERROR("[ros1_in] NOT Served service_2_in");
}
