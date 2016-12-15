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

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rosidl_tutorials_msgs/msg/contact.hpp"


using namespace std::chrono_literals;

class ContactPublisher : public rclcpp::Node
{
public:
  ContactPublisher()
  : Node("address_book_publisher")
  {
    contact_publisher_ = this->create_publisher<rosidl_tutorials_msgs::msg::Contact>("contact");

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<rosidl_tutorials_msgs::msg::Contact>();

        msg->first_name = "John";
        msg->last_name = "Doe";
        msg->age = 30;
        msg->gender = msg->MALE;
        msg->address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << msg->first_name <<
          "  Last:" << msg->last_name << std::endl;

        contact_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<rosidl_tutorials_msgs::msg::Contact>::SharedPtr contact_publisher_;
  rclcpp::timer::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<ContactPublisher>();

  rclcpp::spin(publisher_node);

  return 0;
}
