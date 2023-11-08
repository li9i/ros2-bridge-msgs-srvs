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
    ros::Subscriber subscriber_;

    // **** methods

    /***************************************************************************
     * @brief Callback
     * @param[in] msg [const std_msgs::String] The msg
     * @return void
     */
    void callback(const std_msgs::String& msg);
};

#endif
