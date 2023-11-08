
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

/*******************************************************************************
*/
class MinimalPublisher final : public rclcpp::Node
{
  public:
    MinimalPublisher() :
      Node("minimal_publisher")
  {
    RCLCPP_INFO(this->get_logger(), "[ros2_out] Constructor");

    // https://robotics.stackexchange.com/questions/88250/ros2-error-creating-a-service-server-as-a-member-function
    service_ = this->create_service<std_srvs::srv::Trigger>("service_2_out",
      std::bind(&MinimalPublisher::service_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Create callback group for service clients
    cb_grp_client_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create service clients. They belong to the same callback group
    service_client_2_in_to_2_in_  =
      this->create_client<std_srvs::srv::Trigger>("service_2_in",
        rmw_qos_profile_services_default, cb_grp_client_);
    service_client_2_in_to_1_out_  =
      this->create_client<std_srvs::srv::Trigger>("service_1_out",
        rmw_qos_profile_services_default, cb_grp_client_);
    service_client_2_in_to_1_in_   =
      this->create_client<std_srvs::srv::Trigger>("service_1_in",
        rmw_qos_profile_services_default, cb_grp_client_);

    // Create callback group for timers
    cb_grp_timer_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create timers
    timer_2_in_to_2_in_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalPublisher::timer_callback_2_in_to_2_in, this),
      cb_grp_timer_);

    timer_2_in_to_1_in_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalPublisher::timer_callback_2_in_to_1_in, this),
      cb_grp_timer_);

    timer_2_in_to_1_out_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalPublisher::timer_callback_2_in_to_1_out, this),
      cb_grp_timer_);
  }

  private:

    // -------------------------------------------------------------------------
    void timer_callback_2_in_to_2_in()
    {
      while (!service_client_2_in_to_2_in_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_out] client of service_2_in interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_out] waiting for service_2_in to appear...");
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future =
        service_client_2_in_to_2_in_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_out] service_2_in called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_out] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_out] service_2_in returned");
    }

    // -------------------------------------------------------------------------
    void timer_callback_2_in_to_1_out()
    {
      while (!service_client_2_in_to_1_out_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_out] client of service_1_out interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_out] waiting for service_1_out to appear...");
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future =
        service_client_2_in_to_1_out_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_out] service_1_out called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_out] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_out] service_1_out returned");
    }

    // -------------------------------------------------------------------------
    void timer_callback_2_in_to_1_in()
    {
      while (!service_client_2_in_to_1_in_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_out] client of service_1_in interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_out] waiting for service_1_in to appear...");
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future =
        service_client_2_in_to_1_in_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_out] service_1_in called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_out] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_out] service_1_in returned");
    }



    // -------------------------------------------------------------------------
    void service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      res->success = 1;
      res->message = "ros2_out_success_message";
      RCLCPP_INFO(this->get_logger(), "[ros2_out] Service called");
    }

    //--------------------------------------------------------------------------
    // Callback groups
    rclcpp::CallbackGroup::CallbackGroup::SharedPtr cb_grp_client_;
    rclcpp::CallbackGroup::CallbackGroup::SharedPtr cb_grp_timer_;

    rclcpp::TimerBase::SharedPtr timer_2_in_to_2_in_;
    rclcpp::TimerBase::SharedPtr timer_2_in_to_1_in_;
    rclcpp::TimerBase::SharedPtr timer_2_in_to_1_out_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_2_in_to_2_in_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_2_in_to_1_in_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_2_in_to_1_out_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};


/*******************************************************************************
*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto service_caller = std::make_shared<MinimalPublisher>();
  exec.add_node(service_caller);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
