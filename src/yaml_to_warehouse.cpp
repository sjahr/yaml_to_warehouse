/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mark Moll */

#include <fmt/core.h>

#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>

#include <warehouse_ros/database_connection.h>
#include <rclcpp/rclcpp.hpp>

#include <yaml_msg_convert.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("yaml_to_warehouse");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("yaml_to_warehouse", node_options);

  // clang-format off
  boost::program_options::options_description desc;
  desc.add_options()
      ("help", "Show help message")
      ("directory", boost::program_options::value<std::string>()->default_value("/home/mmoll/ws_benchmarkdb/src/yaml_to_warehouse/data"),
                    "Name of directory containing motion planning benchmarks.")
      ("num", boost::program_options::value<unsigned int>()->default_value(10u),
              "Number of motion planning benchmarks to process.")
      ("host", boost::program_options::value<std::string>()->default_value("/tmp/test_db.sqlite"), "Host for the DB.")
      ("port", boost::program_options::value<std::size_t>(), "Port for the DB.");
  // clang-format on

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc - 4, argv + 4, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1)  // show help if no parameters passed
  {
    std::cout << desc << '\n';
    return 1;
  }
  // Set up db
  node->set_parameter(rclcpp::Parameter("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection"));
  if (vm.count("host"))
  {
    node->set_parameter(rclcpp::Parameter("warehouse_host", vm["host"].as<std::string>()));
  }

  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase(node);
  if (!conn->connect())
    return 1;

  //   planning_scene_monitor::PlanningSceneMonitor psm(node, ROBOT_DESCRIPTION);
  //   if (!psm.getPlanningScene())
  //   {
  //     RCLCPP_ERROR(LOGGER, "Unable to initialize PlanningSceneMonitor");
  //     return 1;
  //   }

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_msgs::msg::PlanningScene scene_msg;
  moveit_msgs::msg::MotionPlanRequest request_msg;
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  unsigned num = vm["num"].as<unsigned int>();
  std::string path = vm["directory"].as<std::string>();

  for (unsigned i = 1; i <= num; ++i)
  {
    std::string scene = fmt::format("{}/scene{:04}.yaml", path, i);
    std::string scene_sensed = fmt::format("{}/scene_sensed{:04}.yaml", path, i);
    std::string request = fmt::format("{}/request{:04}.yaml", path, i);
    std::string trajectory = fmt::format("{}/trajectory{:04}.yaml", path, i);

    if (yaml_msg::fromYAMLFile(scene_msg, scene))
    {
      scene_msg.name = fmt::format("scene{}", i);
    }
    if (yaml_msg::fromYAMLFile(scene_msg, scene_sensed))
    {
      scene_msg.name = fmt::format("scene_sensed{}", i);
    }
    pss.addPlanningScene(scene_msg);
    if (yaml_msg::fromYAMLFile(request_msg, request) && yaml_msg::fromYAMLFile(trajectory_msg, trajectory))
    {
      pss.addPlanningQuery(request_msg, scene_msg.name, scene_msg.name + "_query");
      pss.addPlanningResult(request_msg, trajectory_msg, scene);
    }
    std::cout << scene_msg.name << '\n' << scene_sensed << '\n' << request << '\n' << trajectory << std::endl;
    std::cout << "================================================================================================="
              << std::endl;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
