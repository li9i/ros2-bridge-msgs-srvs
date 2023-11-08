#ifndef ROS1_LISTENER_H
#define ROS1_LISTENER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

class R1L
{
  public:

    R1L(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~R1L();

  private:

    // **** ros

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // subscribers
    ros::Subscriber subscriber_1_in_to_1_out_;
    ros::Subscriber subscriber_2_in_to_1_out_;
    ros::Subscriber subscriber_2_out_to_1_out_;

    ros::Publisher publisher_1_out_to_1_in_;
    ros::Publisher publisher_1_out_to_2_in_;
    ros::Publisher publisher_1_out_to_2_out_;

    ros::Timer timer_1_out_to_1_in_;
    ros::Timer timer_1_out_to_2_in_;
    ros::Timer timer_1_out_to_2_out_;

    // **** methods

    // subscriber callbacks
    void callback_1_in_to_1_out (const std_msgs::String& msg);
    void callback_2_in_to_1_out (const std_msgs::String& msg);
    void callback_2_out_to_1_out(const std_msgs::String& msg);

    // timer/publisher callbacks
    void timer_1_out_to_1_in_callback (const ros::TimerEvent& e);
    void timer_1_out_to_2_in_callback (const ros::TimerEvent& e);
    void timer_1_out_to_2_out_callback(const ros::TimerEvent& e);
};

#endif
