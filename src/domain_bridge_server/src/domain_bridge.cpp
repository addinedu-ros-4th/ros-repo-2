// Copyright 2021, Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "domain_bridge/component_manager.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/parse_domain_bridge_yaml_config.hpp"
#include "domain_bridge/process_cmd_line_arguments.hpp"
#include "task_msgs/srv/allocate_task.hpp"
int SERVER_ID = 212;
int ROBOT_1_ID = 213;
int ROBOT_2_ID = 214;
int ROBOT_3_ID = 215;
int main(int argc, char ** argv)
{
  auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto config_rc_pair = domain_bridge::process_cmd_line_arguments(arguments);
  if (!config_rc_pair.first || 0 != config_rc_pair.second) {
    return config_rc_pair.second;
  }
  domain_bridge::DomainBridge domain_bridge(*config_rc_pair.first);

  // Add component manager node and domain bridge to single-threaded executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<domain_bridge::ComponentManager>(executor);

  domain_bridge.bridge_service<task_msgs::srv::AllocateTask>(("task_" + std::to_string(ROBOT_1_ID)), ROBOT_1_ID, SERVER_ID);
  domain_bridge.bridge_service<task_msgs::srv::AllocateTask>(("task_" + std::to_string(ROBOT_2_ID)), ROBOT_2_ID, SERVER_ID);
  domain_bridge.bridge_service<task_msgs::srv::AllocateTask>(("task_" + std::to_string(ROBOT_3_ID)), ROBOT_3_ID, SERVER_ID);
  

  domain_bridge.add_to_executor(*executor);
  executor->add_node(node);

  

  // domain_bridge::ServiceBridgeOptions options;
  
  



  executor->spin();

  rclcpp::shutdown();
  return 0;
}