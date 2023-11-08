#ifndef ROS1_LISTENER_H
#define ROS1_LISTENER_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class R1L
{
  public:

    R1L(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~R1L();

  private:

    // **** ros

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // service server
    ros::ServiceServer service_server_;

    // service client
    ros::ServiceClient service_client_1_out_to_1_in_;
    ros::ServiceClient service_client_1_out_to_2_in_;
    ros::ServiceClient service_client_1_out_to_2_out_;

    // timers
    ros::Timer timer_1_out_to_1_in_;
    ros::Timer timer_1_out_to_2_in_;
    ros::Timer timer_1_out_to_2_out_;

    // Service callback
    bool serviceServerCallback(
      std_srvs::Trigger::Request& req,
      std_srvs::Trigger::Response& res);

    // **** methods

    // timer/publisher callbacks
    void timer_1_out_to_1_in_callback (const ros::TimerEvent& e);
    void timer_1_out_to_2_in_callback (const ros::TimerEvent& e);
    void timer_1_out_to_2_out_callback(const ros::TimerEvent& e);

};

#endif
