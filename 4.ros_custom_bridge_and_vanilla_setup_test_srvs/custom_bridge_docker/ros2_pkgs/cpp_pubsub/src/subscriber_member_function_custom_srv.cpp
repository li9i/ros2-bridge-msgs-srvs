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
#include "cpp_pubsub/srv/custom_srv.hpp"

/*******************************************************************************
*/
class MinimalSubscriber final : public rclcpp::Node
{
  public:
    MinimalSubscriber() :
      Node("minimal_subscriber")
  {
    RCLCPP_INFO(this->get_logger(), "[ros2_in] Constructor");

    // https://robotics.stackexchange.com/questions/88250/ros2-error-creating-a-service-server-as-a-member-function
    service_ = this->create_service<std_srvs::srv::Trigger>("service_2_in",
      std::bind(&MinimalSubscriber::service_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Create callback group for service clients
    cb_grp_client_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    // Create service clients. They belong to the same callback group
    service_client_2_in_to_2_out_  =
      this->create_client<std_srvs::srv::Trigger>("service_2_out",
        rmw_qos_profile_services_default, cb_grp_client_);
    service_client_2_in_to_1_out_  =
      this->create_client<std_srvs::srv::Trigger>("service_1_out",
        rmw_qos_profile_services_default, cb_grp_client_);
    service_client_2_in_to_1_in_   =
      this->create_client<std_srvs::srv::Trigger>("service_1_in",
        rmw_qos_profile_services_default, cb_grp_client_);

    service_client_2_in_to_1_in_custom_ =
      this->create_client<cpp_pubsub::srv::CustomSrv>("service_1_in_custom",
        rmw_qos_profile_services_default, cb_grp_client_);


    // Create callback group for timers
    cb_grp_timer_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    // Create timers
    timer_2_in_to_2_out_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalSubscriber::timer_callback_2_in_to_2_out, this),
      cb_grp_timer_);

    timer_2_in_to_1_in_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalSubscriber::timer_callback_2_in_to_1_in, this),
      cb_grp_timer_);

    timer_2_in_to_1_out_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalSubscriber::timer_callback_2_in_to_1_out, this),
      cb_grp_timer_);

    timer_2_in_to_1_in_custom_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MinimalSubscriber::timer_callback_2_in_to_1_in_custom, this),
      cb_grp_timer_);
  }

  private:

    // custom srv callback -----------------------------------------------------
    void timer_callback_2_in_to_1_in_custom()
    {
      RCLCPP_INFO(this->get_logger(),
        "[ros2_in] timer_callback_2_in_to_1_in_custom");
      while (!service_client_2_in_to_1_in_custom_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_in] client of service_1_in_custom interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_in] waiting for service_1_in_custom to appear...");
        return;
      }

      auto request = std::make_shared<cpp_pubsub::srv::CustomSrv::Request>();
      request->num = 12;
      auto result_future =
        service_client_2_in_to_1_in_custom_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_in] service_1_in_custom called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_in] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_in] service_1_in_custom returned");
      RCLCPP_INFO(this->get_logger(), "[ros2_in] stamp: %d",
        result->pose.header.stamp.sec);
    }

    // -------------------------------------------------------------------------
    void timer_callback_2_in_to_2_out()
    {
      while (!service_client_2_in_to_2_out_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_in] client of service_2_out interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_in] waiting for service_2_out to appear...");
        return;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future =
        service_client_2_in_to_2_out_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_in] service_2_out called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_in] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_in] service_2_out returned");
    }

    // -------------------------------------------------------------------------
    void timer_callback_2_in_to_1_out()
    {
      while (!service_client_2_in_to_1_out_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_in] client of service_1_out interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_in] waiting for service_1_out to appear...");
        return;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future =
        service_client_2_in_to_1_out_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_in] service_1_out called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_in] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_in] service_1_out returned");
    }

    // -------------------------------------------------------------------------
    void timer_callback_2_in_to_1_in()
    {
      while (!service_client_2_in_to_1_in_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->get_logger(),
            "[ros2_in] client of service_1_in interrupted while waiting for service to appear");
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "[ros2_in] waiting for service_1_in to appear...");
        return;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future =
        service_client_2_in_to_1_in_->async_send_request(request);

      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "[ros2_in] service_1_in called");
        std::future_status status = result_future.wait_for(
          std::chrono::seconds(1));

        if (status == std::future_status::ready)
          break;
      }

      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "[ros2_in] Program canceled");
        return;
      }

      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "[ros2_in] service_1_in returned");
    }




    // -------------------------------------------------------------------------
    void service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      res->success = 1;
      res->message = "ros2_in_success_message";
      RCLCPP_INFO(this->get_logger(), "[ros2_in] Service called");
    }

    //--------------------------------------------------------------------------
    // Callback groups
    rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp_client_;
    rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp_timer_;

    rclcpp::TimerBase::SharedPtr timer_2_in_to_1_in_;
    rclcpp::TimerBase::SharedPtr timer_2_in_to_1_out_;
    rclcpp::TimerBase::SharedPtr timer_2_in_to_2_out_;
    rclcpp::TimerBase::SharedPtr timer_2_in_to_1_in_custom_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_2_in_to_1_in_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_2_in_to_1_out_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_2_in_to_2_out_;
    rclcpp::Client<cpp_pubsub::srv::CustomSrv>::SharedPtr service_client_2_in_to_1_in_custom_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};


/*******************************************************************************
*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto service_caller = std::make_shared<MinimalSubscriber>();
  exec.add_node(service_caller);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
