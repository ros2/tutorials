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

#include "rosidl_tutorials/msg/address_book.hpp"
#include "rosidl_tutorials_msgs/msg/contact.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<rosidl_tutorials::msg::AddressBook>("address_book");

    auto publish_msg = [this]() -> void {
        auto msg = std::make_shared<rosidl_tutorials::msg::AddressBook>();
        {
          rosidl_tutorials_msgs::msg::Contact contact;
          contact.first_name = "John";
          contact.last_name = "Doe";
          contact.age = 30;
          contact.gender = contact.MALE;
          contact.address = "unknown";
          msg->address_book.push_back(contact);
        }
        {
          rosidl_tutorials_msgs::msg::Contact contact;
          contact.first_name = "Jane";
          contact.last_name = "Doe";
          contact.age = 20;
          contact.gender = contact.FEMALE;
          contact.address = "unknown";
          msg->address_book.push_back(contact);
        }

        std::cout << "Publishing address book:" << std::endl;
        for (auto contact : msg->address_book) {
          std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
            std::endl;
        }

        address_book_publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<rosidl_tutorials::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::timer::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<AddressBookPublisher>();

  rclcpp::spin(publisher_node);

  return 0;
}
