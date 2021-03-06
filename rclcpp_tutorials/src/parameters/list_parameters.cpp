// Copyright 2015 Open Source Robotics Foundation, Inc.
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("list_parameters");

  // TODO(esteve): Make the parameter service automatically start with the node.
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);

  // Set several differnet types of parameters.
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foo.first", 8),
    rclcpp::parameter::ParameterVariant("foo.second", 42),
    rclcpp::parameter::ParameterVariant("foobar", true),
  });

  // List the details of a few parameters up to a namespace depth of 10.
  auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"}, 10);
  for (auto & name : parameters_and_prefixes.names) {
    std::cout << "Parameter name: " << name << std::endl;
  }
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    std::cout << "Parameter prefix: " << prefix << std::endl;
  }

  return 0;
}
