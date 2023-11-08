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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher() : Node("publisher_2_out"), count_(0)
    {
      publisher_2_out_to_2_in_ = this->create_publisher<std_msgs::msg::String>("topic_2_out_to_2_in", 10);
      timer_2_out_to_2_in_ = this->create_wall_timer(
        1s, std::bind(&MinimalPublisher::timer_callback_2_out_to_2_in, this));

      publisher_2_out_to_1_in_ = this->create_publisher<std_msgs::msg::String>("topic_2_out_to_1_in", 10);
      timer_2_out_to_1_in_ = this->create_wall_timer(
        1s, std::bind(&MinimalPublisher::timer_callback_2_out_to_1_in, this));

      publisher_2_out_to_1_out_ = this->create_publisher<std_msgs::msg::String>("topic_2_out_to_1_out", 10);
      timer_2_out_to_1_out_ = this->create_wall_timer(
        1s, std::bind(&MinimalPublisher::timer_callback_2_out_to_1_out, this));
    }

  private:
    void timer_callback_2_out_to_2_in()
    {
      auto message = std_msgs::msg::String();
      message.data = "2_out_to_2_in";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_2_out_to_2_in_->publish(message);
    }
    void timer_callback_2_out_to_1_in()
    {
      auto message = std_msgs::msg::String();
      message.data = "2_out_to_1_in";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_2_out_to_1_in_->publish(message);
    }
    void timer_callback_2_out_to_1_out()
    {
      auto message = std_msgs::msg::String();
      message.data = "2_out_to_1_out";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_2_out_to_1_out_->publish(message);
    }


    rclcpp::TimerBase::SharedPtr timer_2_out_to_2_in_;
    rclcpp::TimerBase::SharedPtr timer_2_out_to_1_in_;
    rclcpp::TimerBase::SharedPtr timer_2_out_to_1_out_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_out_to_2_in_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_out_to_1_in_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_out_to_1_out_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
