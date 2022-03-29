// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <list>
#include <memory>
#include <vector>
#include <set>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "head_tracking_msgs/msg/pan_tilt_command.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "head_tracking_bt/TrackObjects.hpp"
#include "head_tracking/HeadController.hpp"


#include "gtest/gtest.h"

using namespace std::placeholders;
using namespace std::chrono_literals;


TEST(bt_action, patrol_btn)
{
  auto node_head_tracker = std::make_shared<head_tracking::HeadController>();
  auto node_test = rclcpp::Node::make_shared("test_node");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node_head_tracker->get_node_base_interface());
  exe.add_node(node_test);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  head_tracking_msgs::msg::PanTiltCommand last_command;
  auto command_sub = node_test->create_subscription<head_tracking_msgs::msg::PanTiltCommand>(
    "command", 100, [&last_command](const head_tracking_msgs::msg::PanTiltCommand::SharedPtr msg) {
      last_command = *msg;
    });

  trajectory_msgs::msg::JointTrajectory last_joint_command;
  auto joint_command_sub = node_test->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_command", 100,
    [&last_joint_command](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      last_joint_command = *msg;
    });

  rclcpp::Rate rate(10);
  auto start = node_test->now();

  {  // We need a way to destroy tree before joining with the spinning thread, so add a block.
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("track_objects_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <Sequence>
            <TrackObjects    name="track_object" pan="{pan}" tilt="{tilt}"/>
          </Sequence>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node_test);

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    while (rclcpp::ok() && (node_test->now() - start) < 2s && !finish) {
      tree.rootNode()->executeTick();
      rate.sleep();
    }

    ASSERT_EQ(
      node_head_tracker->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    blackboard->set<double>("pan", 0.7);
    blackboard->set<double>("tilt", 0.3);

    start = node_test->now();
    while (rclcpp::ok() && (node_test->now() - start) < 2s && !finish) {
      tree.rootNode()->executeTick();
      rate.sleep();
    }

    ASSERT_EQ(last_command.pan, 0.7);
    ASSERT_EQ(last_command.tilt, 0.3);
    ASSERT_EQ(last_joint_command.points.size(), 1u);
    ASSERT_EQ(last_joint_command.points[0].positions.size(), 2u);
    ASSERT_EQ(last_joint_command.points[0].positions[0], 0.7);
    ASSERT_EQ(last_joint_command.points[0].positions[1], 0.3);
  }

  ASSERT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  start = node_test->now();
  while (rclcpp::ok() && (node_test->now() - start) < 2s && !finish) {
    rate.sleep();
  }

  try {
    finish = true;
    t.join();
  } catch (const std::exception & e) {
    std::cerr << "Error at joining: " << e.what() << std::endl;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
