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
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber() : Node("subscriber_2_out")
    {
      subscription_2_in_to_2_out_ = this->create_subscription<std_msgs::msg::String>(
        "topic_2_in_to_2_out", 10, std::bind(&MinimalSubscriber::topic_callback_2_in_to_2_out, this, _1));

      subscription_1_in_to_2_out_ = this->create_subscription<std_msgs::msg::String>(
        "topic_1_in_to_2_out", 10, std::bind(&MinimalSubscriber::topic_callback_1_in_to_2_out, this, _1));

      subscription_1_out_to_2_out_ = this->create_subscription<std_msgs::msg::String>(
        "topic_1_out_to_2_out", 10, std::bind(&MinimalSubscriber::topic_callback_1_out_to_2_out, this, _1));
    }

  private:
    void topic_callback_2_in_to_2_out(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    void topic_callback_1_in_to_2_out(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    void topic_callback_1_out_to_2_out(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_in_to_2_out_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_in_to_2_out_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_out_to_2_out_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
