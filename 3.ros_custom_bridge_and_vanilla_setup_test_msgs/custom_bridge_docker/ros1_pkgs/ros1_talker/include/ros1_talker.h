#ifndef ROS1_talker_H
#define ROS1_talker_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros1_talker/custom_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class R1T
{
  public:

    R1T(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~R1T();

  private:

    // **** ros

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // subscribers
    ros::Publisher chatter_pub_;
    ros::Publisher custom_chatter_pub_;

    // For communication outside this docker container
    ros::Publisher publisher_1_in_to_2_out_;
    ros::Publisher publisher_1_in_to_1_out_;

    // For communication outside this docker container
    ros::Subscriber subscriber_1_out_to_1_in_;
    ros::Subscriber subscriber_2_out_to_1_in_;

    ros::Timer timer_;
    ros::Timer custom_timer_;

    // For communication outside this docker container
    ros::Timer timer_1_in_to_2_out_;
    ros::Timer timer_1_in_to_1_out_;

    // **** methods

    /***************************************************************************
     * @brief Callback
     * @param[in] msg [const std_msgs::String] The msg
     * @return void
     */
    void callback(const ros::TimerEvent& te);
    void customCallback(const ros::TimerEvent& te);


    // For communication outside this docker container
    void callback_1_in_to_2_out(const ros::TimerEvent& te);
    void callback_1_in_to_1_out(const ros::TimerEvent& te);

    // For communication outside this docker container
    void callback_1_out_to_1_in(const std_msgs::String& msg);
    void callback_2_out_to_1_in(const std_msgs::String& msg);
};

#endif
