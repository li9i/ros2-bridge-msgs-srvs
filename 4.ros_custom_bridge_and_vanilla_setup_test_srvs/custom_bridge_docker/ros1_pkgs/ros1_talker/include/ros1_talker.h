#ifndef ROS1_talker_H
#define ROS1_talker_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ros1_talker/custom_srv.h>
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

    // For communication outside this docker container
    ros::ServiceServer service_server_;
    ros::ServiceServer service_server_custom_;

    ros::ServiceClient service_client_1_in_to_2_in_;
    ros::ServiceClient service_client_1_in_to_2_out_;
    ros::ServiceClient service_client_1_in_to_1_out_;

    ros::Timer timer_;
    ros::Timer custom_timer_;

    // For communication outside this docker container
    ros::Timer timer_1_in_to_2_in_;
    ros::Timer timer_1_in_to_2_out_;
    ros::Timer timer_1_in_to_1_out_;

    // **** methods

    /***************************************************************************
     * @brief Callback
     * @param[in] msg [const std_msgs::String] The msg
     * @return void
     */
    bool serviceServerCallback(
      std_srvs::Trigger::Request& req,
      std_srvs::Trigger::Response& res);
    bool serviceServerCustomCallback(
        ros1_talker::custom_srv::Request& req,
        ros1_talker::custom_srv::Response& res);

    // For calls to other nodes' services
    void timer_1_in_to_1_out_callback(const ros::TimerEvent& te);
    void timer_1_in_to_2_out_callback(const ros::TimerEvent& te);
    void timer_1_in_to_2_in_callback (const ros::TimerEvent& te);
};

#endif
