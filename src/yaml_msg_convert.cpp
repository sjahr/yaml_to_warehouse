/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Zachary Kingston */
// See https://github.com/KavrakiLab/robowflex/blob/master/robowflex_library/src/yaml.cpp

#include <cstdint>

#include <algorithm>
#include <string>

#include <yaml-cpp/yaml.h>

#include <boost/algorithm/hex.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <geometry.h>
#include <yaml_msg_convert.h>

#include <rclcpp/rclcpp.hpp>

#define YAML_FLOW(n) n.SetStyle(YAML::EmitterStyle::Flow);

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("yaml_to_warehouse");

static std::string boolToString(bool b)
{
  return b ? "true" : "false";
}

static bool nodeToBool(const YAML::Node& n)
{
  std::string s = n.as<std::string>();
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s == "true";
}

static bool isHeaderEmpty(const std_msgs::msg::Header& h)
{
  return h.stamp.sec == 0l && h.stamp.nanosec == 0ul && h.frame_id == "world";
}

static std_msgs::msg::Header getDefaultHeader()
{
  std_msgs::msg::Header msg;
  msg.frame_id = "world";
  return msg;
}

static unsigned int nodeToCollisionObject(const YAML::Node& n)
{
  try
  {
    std::string s = n.as<std::string>();
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

    if (s == "move")
      return moveit_msgs::msg::CollisionObject::MOVE;
    if (s == "remove")
      return moveit_msgs::msg::CollisionObject::REMOVE;
    if (s == "append")
      return moveit_msgs::msg::CollisionObject::APPEND;

    return moveit_msgs::msg::CollisionObject::ADD;
  }
  catch (const YAML::BadConversion& e)
  {
    // Sometimes it is specified as the int.
    int op = n.as<int>();
    switch (op)
    {
      case 0:
        return moveit_msgs::msg::CollisionObject::ADD;
      case 1:
        return moveit_msgs::msg::CollisionObject::REMOVE;
      case 2:
        return moveit_msgs::msg::CollisionObject::APPEND;
      case 3:
        return moveit_msgs::msg::CollisionObject::MOVE;
      default:
        return moveit_msgs::msg::CollisionObject::ADD;
    }
  }
}

static std::string primitiveTypeToString(const shape_msgs::msg::SolidPrimitive& shape)
{
  switch (shape.type)
  {
    case shape_msgs::msg::SolidPrimitive::BOX:
      return "box";
      break;
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      return "sphere";
      break;
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      return "cylinder";
      break;
    case shape_msgs::msg::SolidPrimitive::CONE:
      return "cone";
      break;
    default:
      return "invalid";
      break;
  }
}

static void nodeToPrimitiveType(const YAML::Node& n, shape_msgs::msg::SolidPrimitive& shape)
{
  std::string s = n.as<std::string>();
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);

  if (s == "sphere")
    shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  else if (s == "cylinder")
    shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  else if (s == "cone")
    shape.type = shape_msgs::msg::SolidPrimitive::CONE;
  else if (s == "box")
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  else
    shape.type = n.as<int>();
}

static bool isVector3Zero(const geometry_msgs::msg::Vector3& v)
{
  return v.x == 0 && v.y == 0 && v.z == 0;
}

static bool isConstraintEmpty(const moveit_msgs::msg::Constraints& c)
{
  return c.joint_constraints.empty()           //
         && c.position_constraints.empty()     //
         && c.orientation_constraints.empty()  //
         && c.visibility_constraints.empty();
}

static std::string compressHex(const std::vector<int8_t>& v)
{
  std::vector<char> compress;
  {
    boost::iostreams::filtering_ostream fos;
    fos.push(boost::iostreams::zlib_compressor());
    fos.push(boost::iostreams::back_inserter(compress));

    for (const auto& i : v)
      fos << i;
  }

  std::string result;
  boost::algorithm::hex(compress.begin(), compress.end(), std::back_inserter(result));

  return result;
}

static std::vector<int8_t> decompressHex(const std::string& hex)
{
  std::vector<int8_t> unhexed;
  boost::algorithm::unhex(hex, std::back_inserter(unhexed));

  std::vector<char> decompress;
  {
    boost::iostreams::filtering_ostream fos;
    fos.push(boost::iostreams::zlib_decompressor());
    fos.push(boost::iostreams::back_inserter(decompress));

    for (const auto& i : unhexed)
      fos << i;
  }

  return std::vector<int8_t>(decompress.begin(), decompress.end());
}
}  // namespace

namespace YAML
{
Node convert<moveit_msgs::msg::PlanningScene>::encode(const moveit_msgs::msg::PlanningScene& rhs)
{
  Node node;
  node["name"] = rhs.name;
  node["robot_state"] = rhs.robot_state;
  node["robot_model_name"] = rhs.robot_model_name;
  node["fixed_frame_transforms"] = rhs.fixed_frame_transforms;
  node["allowed_collision_matrix"] = rhs.allowed_collision_matrix;

  // node["link_padding"] = rhs.link_padding;
  // node["link_padding"].SetStyle(YAML::EmitterStyle::Flow);

  // node["link_scale"] = rhs.link_scale;
  // node["link_scale"].SetStyle(YAML::EmitterStyle::Flow);

  // node["object_colors"] = rhs.object_colors;

  if (!rhs.world.collision_objects.empty() || !rhs.world.octomap.octomap.data.empty())
    node["world"] = rhs.world;

  if (rhs.is_diff)
    node["is_diff"] = boolToString(rhs.is_diff);

  return node;
}

bool convert<moveit_msgs::msg::PlanningScene>::decode(const Node& node, moveit_msgs::msg::PlanningScene& rhs)
{
  rhs = moveit_msgs::msg::PlanningScene();

  if (yaml_msg::isNode(node["name"]))
  {
    RCLCPP_INFO(LOGGER, "Decode PlanningScene '%s'", node["name"].as<std::string>().c_str());
    rhs.name = node["name"].as<std::string>();
  }

  if (yaml_msg::isNode(node["robot_state"]))
    rhs.robot_state = node["robot_state"].as<moveit_msgs::msg::RobotState>();

  if (yaml_msg::isNode(node["robot_model_name"]))
    rhs.robot_model_name = node["robot_model_name"].as<std::string>();

  if (yaml_msg::isNode(node["fixed_frame_transforms"]))
    rhs.fixed_frame_transforms = node["fixed_frame_transforms"].as<std::vector<geometry_msgs::msg::TransformStamped>>();

  if (yaml_msg::isNode(node["allowed_collision_matrix"]))
    rhs.allowed_collision_matrix = node["allowed_collision_matrix"].as<moveit_msgs::msg::AllowedCollisionMatrix>();

  if (yaml_msg::isNode(node["link_padding"]))
    rhs.link_padding = node["link_padding"].as<std::vector<moveit_msgs::msg::LinkPadding>>();

  if (yaml_msg::isNode(node["link_scale"]))
    rhs.link_scale = node["link_scale"].as<std::vector<moveit_msgs::msg::LinkScale>>();

  if (yaml_msg::isNode(node["object_colors"]))
    rhs.object_colors = node["object_colors"].as<std::vector<moveit_msgs::msg::ObjectColor>>();

  if (yaml_msg::isNode(node["world"]))
    rhs.world = node["world"].as<moveit_msgs::msg::PlanningSceneWorld>();

  if (yaml_msg::isNode(node["is_diff"]))
    rhs.is_diff = nodeToBool(node["is_diff"]);

  return true;
}

Node convert<moveit_msgs::msg::RobotState>::encode(const moveit_msgs::msg::RobotState& rhs)
{
  Node node;

  if (!rhs.joint_state.name.empty())
    node["joint_state"] = rhs.joint_state;

  if (!rhs.multi_dof_joint_state.joint_names.empty())
    node["multi_dof_joint_state"] = rhs.multi_dof_joint_state;

  if (!rhs.attached_collision_objects.empty())
    node["attached_collision_objects"] = rhs.attached_collision_objects;

  if (rhs.is_diff)
    node["is_diff"] = boolToString(rhs.is_diff);

  return node;
}

bool convert<moveit_msgs::msg::RobotState>::decode(const Node& node, moveit_msgs::msg::RobotState& rhs)
{
  rhs = moveit_msgs::msg::RobotState();

  if (yaml_msg::isNode(node["joint_state"]))
    rhs.joint_state = node["joint_state"].as<sensor_msgs::msg::JointState>();

  if (yaml_msg::isNode(node["multi_dof_joint_state"]))
    rhs.multi_dof_joint_state = node["multi_dof_joint_state"].as<sensor_msgs::msg::MultiDOFJointState>();

  if (yaml_msg::isNode(node["attached_collision_objects"]))
    rhs.attached_collision_objects =
        node["attached_collision_objects"].as<std::vector<moveit_msgs::msg::AttachedCollisionObject>>();

  if (yaml_msg::isNode(node["is_diff"]))
    rhs.is_diff = nodeToBool(node["is_diff"]);

  return true;
}

Node convert<geometry_msgs::msg::TransformStamped>::encode(const geometry_msgs::msg::TransformStamped& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["child_frame_id"] = rhs.child_frame_id;
  node["transform"] = rhs.transform;
  return node;
}

bool convert<geometry_msgs::msg::TransformStamped>::decode(const Node& node, geometry_msgs::msg::TransformStamped& rhs)
{
  rhs = geometry_msgs::msg::TransformStamped();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["child_frame_id"]))
    rhs.child_frame_id = node["child_frame_id"].as<std::string>();

  if (yaml_msg::isNode(node["transform"]))
    rhs.transform = node["transform"].as<geometry_msgs::msg::Transform>();

  return true;
}

Node convert<std_msgs::msg::Header>::encode(const std_msgs::msg::Header& rhs)
{
  Node node;
  if (!(rhs.stamp.sec == 0l && rhs.stamp.nanosec == 0ul))
  {
    node["stamp"]["secs"] = rhs.stamp.sec;
    node["stamp"]["nsecs"] = rhs.stamp.nanosec;
  }

  if (rhs.frame_id != "world" && rhs.frame_id != "/world")
    node["frame_id"] = rhs.frame_id;

  return node;
}

bool convert<std_msgs::msg::Header>::decode(const Node& node, std_msgs::msg::Header& rhs)
{
  rhs = std_msgs::msg::Header();
  rhs.frame_id = "world";

  if (yaml_msg::isNode(node["stamp"]))
  {
    try
    {
      rhs.stamp.sec = node["stamp"]["sec"].as<int>();
      rhs.stamp.nanosec = node["stamp"]["nsec"].as<int>();
    }
    catch (YAML::InvalidNode& e)
    {
      rhs.stamp.sec = node["stamp"]["secs"].as<int>();
      rhs.stamp.nanosec = node["stamp"]["nsecs"].as<int>();
    }
  }

  if (yaml_msg::isNode(node["frame_id"]))
    rhs.frame_id = node["frame_id"].as<std::string>();

  return true;
}

Node convert<geometry_msgs::msg::Pose>::encode(const geometry_msgs::msg::Pose& rhs)
{
  Node node;
  node["position"] = rhs.position;
  node["orientation"] = rhs.orientation;
  return node;
}

bool convert<geometry_msgs::msg::Pose>::decode(const Node& node, geometry_msgs::msg::Pose& rhs)
{
  rhs = geometry_msgs::msg::Pose();

  if (yaml_msg::isNode(node["position"]))
    rhs.position = node["position"].as<geometry_msgs::msg::Point>();

  if (yaml_msg::isNode(node["orientation"]))
    rhs.orientation = node["orientation"].as<geometry_msgs::msg::Quaternion>();

  return true;
}

Node convert<geometry_msgs::msg::PoseStamped>::encode(const geometry_msgs::msg::PoseStamped& rhs)
{
  Node node;
  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["pose"] = rhs.pose;

  return node;
}

bool convert<geometry_msgs::msg::PoseStamped>::decode(const Node& node, geometry_msgs::msg::PoseStamped& rhs)
{
  rhs = geometry_msgs::msg::PoseStamped();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  rhs.pose = node["pose"].as<geometry_msgs::msg::Pose>();

  return true;
}

Node convert<geometry_msgs::msg::Transform>::encode(const geometry_msgs::msg::Transform& rhs)
{
  Node node;
  node["translation"] = rhs.translation;
  node["rotation"] = rhs.rotation;
  return node;
}

bool convert<geometry_msgs::msg::Transform>::decode(const Node& node, geometry_msgs::msg::Transform& rhs)
{
  rhs = geometry_msgs::msg::Transform();

  if (yaml_msg::isNode(node["translation"]))
    rhs.translation = node["translation"].as<geometry_msgs::msg::Vector3>();

  if (yaml_msg::isNode(node["rotation"]))
    rhs.rotation = node["rotation"].as<geometry_msgs::msg::Quaternion>();

  return true;
}

Node convert<geometry_msgs::msg::Point>::encode(const geometry_msgs::msg::Point& rhs)
{
  Node node;
  YAML_FLOW(node);

  node.push_back(rhs.x);
  node.push_back(rhs.y);
  node.push_back(rhs.z);
  return node;
}

bool convert<geometry_msgs::msg::Point>::decode(const Node& node, geometry_msgs::msg::Point& rhs)
{
  rhs = geometry_msgs::msg::Point();

  if (node.IsSequence())
  {
    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
  }
  else
  {
    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();
    rhs.z = node["z"].as<double>();
  }
  return true;
}

Node convert<geometry_msgs::msg::Vector3>::encode(const geometry_msgs::msg::Vector3& rhs)
{
  Node node;
  YAML_FLOW(node);

  node.push_back(rhs.x);
  node.push_back(rhs.y);
  node.push_back(rhs.z);
  return node;
}

bool convert<geometry_msgs::msg::Vector3>::decode(const Node& node, geometry_msgs::msg::Vector3& rhs)
{
  rhs = geometry_msgs::msg::Vector3();

  if (node.IsSequence())
  {
    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
  }
  else
  {
    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();
    rhs.z = node["z"].as<double>();
  }

  return true;
}

Node convert<geometry_msgs::msg::Quaternion>::encode(const geometry_msgs::msg::Quaternion& rhs)
{
  Node node;
  YAML_FLOW(node);

  node.push_back(rhs.x);
  node.push_back(rhs.y);
  node.push_back(rhs.z);
  node.push_back(rhs.w);
  return node;
}

bool convert<geometry_msgs::msg::Quaternion>::decode(const Node& node, geometry_msgs::msg::Quaternion& rhs)
{
  rhs = geometry_msgs::msg::Quaternion();

  if (node.IsSequence())
  {
    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    rhs.w = node[3].as<double>();
  }
  else
  {
    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();
    rhs.z = node["z"].as<double>();
    rhs.w = node["w"].as<double>();
  }
  return true;
}

Node convert<geometry_msgs::msg::Twist>::encode(const geometry_msgs::msg::Twist& rhs)
{
  Node node;
  node["linear"] = rhs.linear;
  node["angular"] = rhs.angular;
  return node;
}

bool convert<geometry_msgs::msg::Twist>::decode(const Node& node, geometry_msgs::msg::Twist& rhs)
{
  rhs = geometry_msgs::msg::Twist();

  rhs.linear = node["linear"].as<geometry_msgs::msg::Vector3>();
  rhs.angular = node["angular"].as<geometry_msgs::msg::Vector3>();

  return true;
}

Node convert<geometry_msgs::msg::Wrench>::encode(const geometry_msgs::msg::Wrench& rhs)
{
  Node node;
  node["force"] = rhs.force;
  node["torque"] = rhs.torque;
  return node;
}

bool convert<geometry_msgs::msg::Wrench>::decode(const Node& node, geometry_msgs::msg::Wrench& rhs)
{
  rhs = geometry_msgs::msg::Wrench();

  rhs.force = node["force"].as<geometry_msgs::msg::Vector3>();
  rhs.torque = node["torque"].as<geometry_msgs::msg::Vector3>();

  return true;
}

Node convert<sensor_msgs::msg::JointState>::encode(const sensor_msgs::msg::JointState& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  if (!rhs.name.empty())
  {
    node["name"] = rhs.name;
    YAML_FLOW(node["name"]);
  }

  if (!rhs.position.empty())
  {
    node["position"] = rhs.position;
    YAML_FLOW(node["position"]);
  }

  if (!rhs.velocity.empty())
  {
    node["velocity"] = rhs.velocity;
    YAML_FLOW(node["velocity"]);
  }

  if (!rhs.effort.empty())
  {
    node["effort"] = rhs.effort;
    YAML_FLOW(node["effort"]);
  }

  return node;
}

bool convert<sensor_msgs::msg::JointState>::decode(const Node& node, sensor_msgs::msg::JointState& rhs)
{
  rhs = sensor_msgs::msg::JointState();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["name"]))
    rhs.name = node["name"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["position"]))
    rhs.position = node["position"].as<std::vector<double>>();

  if (yaml_msg::isNode(node["velocity"]))
    rhs.velocity = node["velocity"].as<std::vector<double>>();

  if (yaml_msg::isNode(node["effort"]))
    rhs.effort = node["effort"].as<std::vector<double>>();

  return true;
}

Node convert<sensor_msgs::msg::MultiDOFJointState>::encode(const sensor_msgs::msg::MultiDOFJointState& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["joint_names"] = rhs.joint_names;
  YAML_FLOW(node["joint_names"]);

  node["transforms"] = rhs.transforms;
  YAML_FLOW(node["transforms"]);

  node["twist"] = rhs.twist;
  YAML_FLOW(node["twist"]);

  node["wrench"] = rhs.wrench;
  YAML_FLOW(node["wrench"]);
  return node;
}

bool convert<sensor_msgs::msg::MultiDOFJointState>::decode(const Node& node, sensor_msgs::msg::MultiDOFJointState& rhs)
{
  rhs = sensor_msgs::msg::MultiDOFJointState();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["joint_names"]))
    rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["transforms"]))
    rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::msg::Transform>>();

  if (yaml_msg::isNode(node["twist"]))
    rhs.twist = node["twist"].as<std::vector<geometry_msgs::msg::Twist>>();

  if (yaml_msg::isNode(node["wrench"]))
    rhs.wrench = node["wrench"].as<std::vector<geometry_msgs::msg::Wrench>>();

  return true;
}

Node convert<moveit_msgs::msg::AttachedCollisionObject>::encode(const moveit_msgs::msg::AttachedCollisionObject& rhs)
{
  Node node;
  node["link_name"] = rhs.link_name;
  node["object"] = rhs.object;

  if (!rhs.touch_links.empty())
    node["touch_links"] = rhs.touch_links;

  if (!rhs.detach_posture.points.empty())
    node["detach_posture"] = rhs.detach_posture;

  node["weight"] = rhs.weight;
  return node;
}

bool convert<moveit_msgs::msg::AttachedCollisionObject>::decode(const Node& node,
                                                                moveit_msgs::msg::AttachedCollisionObject& rhs)
{
  rhs = moveit_msgs::msg::AttachedCollisionObject();

  if (yaml_msg::isNode(node["link_name"]))
    rhs.link_name = node["link_name"].as<std::string>();

  if (yaml_msg::isNode(node["object"]))
    rhs.object = node["object"].as<moveit_msgs::msg::CollisionObject>();

  if (yaml_msg::isNode(node["touch_links"]))
    rhs.touch_links = node["touch_links"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["detach_posture"]))
    rhs.detach_posture = node["detach_posture"].as<trajectory_msgs::msg::JointTrajectory>();

  if (yaml_msg::isNode(node["weight"]))
    rhs.weight = node["weight"].as<double>();

  return true;
}

Node convert<trajectory_msgs::msg::JointTrajectory>::encode(const trajectory_msgs::msg::JointTrajectory& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["joint_names"] = rhs.joint_names;
  YAML_FLOW(node["joint_names"]);
  node["points"] = rhs.points;
  return node;
}

bool convert<trajectory_msgs::msg::JointTrajectory>::decode(const Node& node, trajectory_msgs::msg::JointTrajectory& rhs)
{
  rhs = trajectory_msgs::msg::JointTrajectory();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["joint_names"]))
    rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["points"]))
    rhs.points = node["points"].as<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>>();

  return true;
}

Node convert<trajectory_msgs::msg::JointTrajectoryPoint>::encode(const trajectory_msgs::msg::JointTrajectoryPoint& rhs)
{
  Node node;

  if (!rhs.positions.empty())
  {
    node["positions"] = rhs.positions;
    YAML_FLOW(node["positions"]);
  }

  if (!rhs.velocities.empty())
  {
    node["velocities"] = rhs.velocities;
    YAML_FLOW(node["velocities"]);
  }

  if (!rhs.accelerations.empty())
  {
    node["accelerations"] = rhs.accelerations;
    YAML_FLOW(node["accelerations"]);
  }

  if (!rhs.effort.empty())
  {
    node["effort"] = rhs.effort;
    YAML_FLOW(node["effort"]);
  }

  node["time_from_start"] = (double)rhs.time_from_start.sec + rhs.time_from_start.nanosec * 1e-9;

  return node;
}

bool convert<trajectory_msgs::msg::JointTrajectoryPoint>::decode(const Node& node,
                                                                 trajectory_msgs::msg::JointTrajectoryPoint& rhs)
{
  if (yaml_msg::isNode(node["positions"]))
    rhs.positions = node["positions"].as<std::vector<double>>();

  if (yaml_msg::isNode(node["velocities"]))
    rhs.velocities = node["velocities"].as<std::vector<double>>();

  if (yaml_msg::isNode(node["accelerations"]))
    rhs.accelerations = node["accelerations"].as<std::vector<double>>();

  if (yaml_msg::isNode(node["effort"]))
    rhs.effort = node["effort"].as<std::vector<double>>();

  if (yaml_msg::isNode(node["time_from_start"]))
  {
    double duration = node["time_from_start"].as<double>();
    double sec = std::floor(duration);
    unsigned long nanosec = (duration - sec) * 1e9;
    rhs.time_from_start = builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec((int)sec).nanosec(nanosec);
  }
  return true;
}

Node convert<moveit_msgs::msg::CollisionObject>::encode(const moveit_msgs::msg::CollisionObject& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["id"] = rhs.id;
  node["pose"] = rhs.pose;

  if (!rhs.type.key.empty())
    node["type"] = rhs.type;

  if (!rhs.primitives.empty())
  {
    node["primitives"] = rhs.primitives;
    node["primitive_poses"] = rhs.primitive_poses;
  }

  if (!rhs.meshes.empty())
  {
    node["meshes"] = rhs.meshes;
    node["mesh_poses"] = rhs.mesh_poses;
  }

  if (!rhs.planes.empty())
  {
    node["planes"] = rhs.planes;
    node["plane_poses"] = rhs.plane_poses;
  }

  std::string s;
  switch (rhs.operation)
  {
    case moveit_msgs::msg::CollisionObject::REMOVE:
      s = "remove";
      break;
    case moveit_msgs::msg::CollisionObject::APPEND:
      s = "append";
      break;
    case moveit_msgs::msg::CollisionObject::MOVE:
      s = "move";
      break;
    default:
      return node;
  }

  node["operation"] = s;
  return node;
}

bool convert<moveit_msgs::msg::CollisionObject>::decode(const Node& node, moveit_msgs::msg::CollisionObject& rhs)
{
  geometry_msgs::msg::Pose pose_msg;
  rhs = moveit_msgs::msg::CollisionObject();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["id"]))
    rhs.id = node["id"].as<std::string>();

  if (yaml_msg::isNode(node["pose"]))
    pose_msg = node["pose"].as<geometry_msgs::msg::Pose>();

  rhs.pose = pose_msg;

  if (yaml_msg::isNode(node["type"]))
    rhs.type = node["type"].as<object_recognition_msgs::msg::ObjectType>();

  if (yaml_msg::isNode(node["primitives"]))
  {
    rhs.primitives = node["primitives"].as<std::vector<shape_msgs::msg::SolidPrimitive>>();
    rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::msg::Pose>>();
  }

  if (yaml_msg::isNode(node["meshes"]))
  {
    rhs.meshes = node["meshes"].as<std::vector<shape_msgs::msg::Mesh>>();
    rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::msg::Pose>>();
  }

  if (yaml_msg::isNode(node["planes"]))
  {
    rhs.planes = node["planes"].as<std::vector<shape_msgs::msg::Plane>>();
    rhs.plane_poses = node["plane_poses"].as<std::vector<geometry_msgs::msg::Pose>>();
  }

  if (yaml_msg::isNode(node["operation"]))
    rhs.operation = nodeToCollisionObject(node["operation"]);

  return true;
}

Node convert<object_recognition_msgs::msg::ObjectType>::encode(const object_recognition_msgs::msg::ObjectType& rhs)
{
  Node node;
  node["key"] = rhs.key;
  node["db"] = rhs.db;
  return node;
}

bool convert<object_recognition_msgs::msg::ObjectType>::decode(const Node& node,
                                                               object_recognition_msgs::msg::ObjectType& rhs)
{
  rhs = object_recognition_msgs::msg::ObjectType();

  if (yaml_msg::isNode(node["key"]))
    rhs.key = node["key"].as<std::string>();

  if (yaml_msg::isNode(node["db"]))
    rhs.db = node["db"].as<std::string>();

  return true;
}

Node convert<moveit_msgs::msg::LinkPadding>::encode(const moveit_msgs::msg::LinkPadding& rhs)
{
  Node node;
  node["link_name"] = rhs.link_name;
  node["padding"] = rhs.padding;
  return node;
}

bool convert<moveit_msgs::msg::LinkPadding>::decode(const Node& node, moveit_msgs::msg::LinkPadding& rhs)
{
  rhs = moveit_msgs::msg::LinkPadding();

  if (yaml_msg::isNode(node["link_name"]))
    rhs.link_name = node["link_name"].as<std::string>();

  if (yaml_msg::isNode(node["padding"]))
    rhs.padding = node["padding"].as<double>();

  return true;
}

Node convert<moveit_msgs::msg::LinkScale>::encode(const moveit_msgs::msg::LinkScale& rhs)
{
  Node node;
  node["link_name"] = rhs.link_name;
  node["scale"] = rhs.scale;
  return node;
}

bool convert<moveit_msgs::msg::LinkScale>::decode(const Node& node, moveit_msgs::msg::LinkScale& rhs)
{
  rhs = moveit_msgs::msg::LinkScale();

  if (yaml_msg::isNode(node["link_name"]))
    rhs.link_name = node["link_name"].as<std::string>();

  if (yaml_msg::isNode(node["scale"]))
    rhs.scale = node["scale"].as<double>();

  return true;
}

Node convert<moveit_msgs::msg::ObjectColor>::encode(const moveit_msgs::msg::ObjectColor& rhs)
{
  Node node;
  node["id"] = rhs.id;
  node["color"] = rhs.color;
  return node;
}

bool convert<moveit_msgs::msg::ObjectColor>::decode(const Node& node, moveit_msgs::msg::ObjectColor& rhs)
{
  rhs = moveit_msgs::msg::ObjectColor();

  if (yaml_msg::isNode(node["id"]))
    rhs.id = node["id"].as<std::string>();

  if (yaml_msg::isNode(node["color"]))
    rhs.color = node["color"].as<std_msgs::msg::ColorRGBA>();

  return true;
}

Node convert<std_msgs::msg::ColorRGBA>::encode(const std_msgs::msg::ColorRGBA& rhs)
{
  Node node;
  YAML_FLOW(node);

  node.push_back(rhs.r);
  node.push_back(rhs.g);
  node.push_back(rhs.b);
  node.push_back(rhs.a);
  return node;
}

bool convert<std_msgs::msg::ColorRGBA>::decode(const Node& node, std_msgs::msg::ColorRGBA& rhs)
{
  rhs = std_msgs::msg::ColorRGBA();

  rhs.r = node[0].as<double>();
  rhs.g = node[1].as<double>();
  rhs.b = node[2].as<double>();
  rhs.a = node[3].as<double>();
  return true;
}

Node convert<moveit_msgs::msg::AllowedCollisionMatrix>::encode(const moveit_msgs::msg::AllowedCollisionMatrix& rhs)
{
  Node node;
  node["entry_names"] = rhs.entry_names;
  YAML_FLOW(node["entry_names"]);

  node["entry_values"] = rhs.entry_values;

  if (!rhs.default_entry_values.empty())
  {
    node["default_entry_names"] = rhs.entry_names;
    YAML_FLOW(node["default_entry_names"]);

    std::vector<std::string> default_entry_values;
    for (const auto& b : rhs.default_entry_values)
      default_entry_values.emplace_back(boolToString(b));

    node["default_entry_values"] = default_entry_values;
    YAML_FLOW(node["default_entry_values"]);
  }

  return node;
}

bool convert<moveit_msgs::msg::AllowedCollisionMatrix>::decode(const Node& node,
                                                               moveit_msgs::msg::AllowedCollisionMatrix& rhs)
{
  rhs = moveit_msgs::msg::AllowedCollisionMatrix();

  if (yaml_msg::isNode(node["entry_names"]))
    rhs.entry_names = node["entry_names"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["entry_values"]))
    rhs.entry_values = node["entry_values"].as<std::vector<moveit_msgs::msg::AllowedCollisionEntry>>();

  if (yaml_msg::isNode(node["default_entry_names"]))
    rhs.default_entry_names = node["default_entry_names"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["default_entry_values"]))
  {
    const auto& dev = node["default_entry_values"];
    for (const auto& b : dev)
      rhs.default_entry_values.push_back(nodeToBool(b));
  }

  return true;
}

Node convert<moveit_msgs::msg::AllowedCollisionEntry>::encode(const moveit_msgs::msg::AllowedCollisionEntry& rhs)
{
  Node node;
  std::vector<std::string> enabled;
  for (const auto& b : rhs.enabled)
    enabled.emplace_back(boolToString(b));

  node = enabled;
  YAML_FLOW(node);
  return node;
}

bool convert<moveit_msgs::msg::AllowedCollisionEntry>::decode(const Node& node,
                                                              moveit_msgs::msg::AllowedCollisionEntry& rhs)
{
  rhs = moveit_msgs::msg::AllowedCollisionEntry();

  for (const auto& b : node)
    rhs.enabled.push_back(nodeToBool(b));

  return true;
}

Node convert<moveit_msgs::msg::PlanningSceneWorld>::encode(const moveit_msgs::msg::PlanningSceneWorld& rhs)
{
  Node node;

  if (!rhs.collision_objects.empty())
    node["collision_objects"] = rhs.collision_objects;

  if (!rhs.octomap.octomap.data.empty())
    node["octomap"] = rhs.octomap;

  return node;
}

bool convert<moveit_msgs::msg::PlanningSceneWorld>::decode(const Node& node, moveit_msgs::msg::PlanningSceneWorld& rhs)
{
  rhs = moveit_msgs::msg::PlanningSceneWorld();

  if (yaml_msg::isNode(node["collision_objects"]))
    rhs.collision_objects = node["collision_objects"].as<std::vector<moveit_msgs::msg::CollisionObject>>();

  if (yaml_msg::isNode(node["octomap"]))
    rhs.octomap = node["octomap"].as<octomap_msgs::msg::OctomapWithPose>();

  return true;
}

Node convert<octomap_msgs::msg::Octomap>::encode(const octomap_msgs::msg::Octomap& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["binary"] = boolToString(rhs.binary);
  node["id"] = rhs.id;
  node["resolution"] = rhs.resolution;
  node["data"] = compressHex(rhs.data);

  return node;
}

bool convert<octomap_msgs::msg::Octomap>::decode(const Node& node, octomap_msgs::msg::Octomap& rhs)
{
  rhs = octomap_msgs::msg::Octomap();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["binary"]))
    rhs.binary = nodeToBool(node["binary"]);

  if (yaml_msg::isNode(node["id"]))
    rhs.id = node["id"].as<std::string>();

  if (yaml_msg::isNode(node["resolution"]))
    rhs.resolution = node["resolution"].as<double>();

  if (yaml_msg::isNode(node["data"]))
  {
    // Load old octomap formats / direct YAML output
    if (node["data"].IsSequence())
    {
      auto temp = node["data"].as<std::vector<int>>();
      rhs.data = std::vector<int8_t>(temp.begin(), temp.end());
    }
    else
    {
      auto temp = node["data"].as<std::string>();
      rhs.data = decompressHex(temp);
    }
  }

  return true;
}

Node convert<octomap_msgs::msg::OctomapWithPose>::encode(const octomap_msgs::msg::OctomapWithPose& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["origin"] = rhs.origin;
  node["octomap"] = rhs.octomap;
  return node;
}

bool convert<octomap_msgs::msg::OctomapWithPose>::decode(const Node& node, octomap_msgs::msg::OctomapWithPose& rhs)
{
  rhs = octomap_msgs::msg::OctomapWithPose();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  if (yaml_msg::isNode(node["origin"]))
    rhs.origin = node["origin"].as<geometry_msgs::msg::Pose>();

  if (yaml_msg::isNode(node["octomap"]))
    rhs.octomap = node["octomap"].as<octomap_msgs::msg::Octomap>();

  return true;
}

Node convert<rclcpp::Time>::encode(const rclcpp::Time& rhs)
{
  Node node;
  node = rhs.seconds();
  return node;
}

bool convert<rclcpp::Time>::decode(const Node& node, rclcpp::Time& rhs)
{
  rhs = rclcpp::Time(static_cast<unsigned long>(node.as<double>() * 1e9));
  return true;
}

Node convert<rclcpp::Duration>::encode(const rclcpp::Duration& rhs)
{
  Node node;
  node = rhs.seconds();
  return node;
}

bool convert<rclcpp::Duration>::decode(const Node& node, rclcpp::Duration& rhs)
{
  rhs.from_seconds(node.as<double>());
  return true;
}

Node convert<shape_msgs::msg::SolidPrimitive>::encode(const shape_msgs::msg::SolidPrimitive& rhs)
{
  Node node;
  node["type"] = primitiveTypeToString(rhs);
  std::vector<double> dims{ rhs.dimensions[0], rhs.dimensions[1], rhs.dimensions[2] };
  node["dimensions"] = dims;
  YAML_FLOW(node["dimensions"]);
  return node;
}

bool convert<shape_msgs::msg::SolidPrimitive>::decode(const Node& node, shape_msgs::msg::SolidPrimitive& rhs)
{
  rhs = shape_msgs::msg::SolidPrimitive();
  if (yaml_msg::isNode(node["type"]))
    nodeToPrimitiveType(node["type"], rhs);

  if (yaml_msg::isNode(node["dimensions"]))
  {
    auto dims = node["dimensions"].as<std::vector<double>>();
    rhs.dimensions = { dims[0], dims[1], dims[2] };
  }

  return true;
}

Node convert<shape_msgs::msg::Mesh>::encode(const shape_msgs::msg::Mesh& rhs)
{
  Node node;
  node["triangles"] = rhs.triangles;
  node["vertices"] = rhs.vertices;

  return node;
}

bool convert<shape_msgs::msg::Mesh>::decode(const Node& node, shape_msgs::msg::Mesh& rhs)
{
  rhs = shape_msgs::msg::Mesh();
  if (yaml_msg::isNode(node["resource"]))
  {
    std::string resource = node["resource"].as<std::string>();
    Eigen::Vector3d dimensions{ 1, 1, 1 };

    if (yaml_msg::isNode(node["dimensions"]))
    {
      Eigen::Vector3d load(node["dimensions"].as<std::vector<double>>().data());
      dimensions = load;
    }

    Geometry mesh(Geometry::ShapeType::Type::MESH, dimensions, resource);
    rhs = mesh.getMeshMsg();
  }
  else
  {
    if (yaml_msg::isNode(node["triangles"]))
      rhs.triangles = node["triangles"].as<std::vector<shape_msgs::msg::MeshTriangle>>();
    if (yaml_msg::isNode(node["vertices"]))
      rhs.vertices = node["vertices"].as<std::vector<geometry_msgs::msg::Point>>();
  }
  return true;
}

Node convert<shape_msgs::msg::MeshTriangle>::encode(const shape_msgs::msg::MeshTriangle& rhs)
{
  Node node;
  node.push_back(rhs.vertex_indices[0]);
  node.push_back(rhs.vertex_indices[1]);
  node.push_back(rhs.vertex_indices[2]);
  YAML_FLOW(node);
  return node;
}

bool convert<shape_msgs::msg::MeshTriangle>::decode(const Node& node, shape_msgs::msg::MeshTriangle& rhs)
{
  rhs.vertex_indices[0] = node[0].as<double>();
  rhs.vertex_indices[1] = node[1].as<double>();
  rhs.vertex_indices[2] = node[2].as<double>();
  return true;
}

Node convert<shape_msgs::msg::Plane>::encode(const shape_msgs::msg::Plane& rhs)
{
  Node node;
  node["coef"].push_back(rhs.coef[0]);
  node["coef"].push_back(rhs.coef[1]);
  node["coef"].push_back(rhs.coef[2]);
  node["coef"].push_back(rhs.coef[3]);
  YAML_FLOW(node["coef"]);
  return node;
}

bool convert<shape_msgs::msg::Plane>::decode(const Node& node, shape_msgs::msg::Plane& rhs)
{
  rhs.coef[0] = node["coef"][0].as<double>();
  rhs.coef[1] = node["coef"][1].as<double>();
  rhs.coef[2] = node["coef"][2].as<double>();
  rhs.coef[3] = node["coef"][3].as<double>();
  return true;
}

Node convert<moveit_msgs::msg::WorkspaceParameters>::encode(const moveit_msgs::msg::WorkspaceParameters& rhs)
{
  Node node;
  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["min_corner"] = rhs.min_corner;
  node["max_corner"] = rhs.max_corner;
  return node;
}

bool convert<moveit_msgs::msg::WorkspaceParameters>::decode(const Node& node, moveit_msgs::msg::WorkspaceParameters& rhs)
{
  rhs = moveit_msgs::msg::WorkspaceParameters();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  rhs.min_corner = node["min_corner"].as<geometry_msgs::msg::Vector3>();
  rhs.max_corner = node["max_corner"].as<geometry_msgs::msg::Vector3>();
  return true;
}

Node convert<moveit_msgs::msg::Constraints>::encode(const moveit_msgs::msg::Constraints& rhs)
{
  Node node;

  if (!rhs.name.empty())
    node["name"] = rhs.name;

  if (!rhs.joint_constraints.empty())
    node["joint_constraints"] = rhs.joint_constraints;

  if (!rhs.position_constraints.empty())
    node["position_constraints"] = rhs.position_constraints;

  if (!rhs.orientation_constraints.empty())
    node["orientation_constraints"] = rhs.orientation_constraints;

  if (!rhs.visibility_constraints.empty())
    node["visibility_constraints"] = rhs.visibility_constraints;

  return node;
}

bool convert<moveit_msgs::msg::Constraints>::decode(const Node& node, moveit_msgs::msg::Constraints& rhs)
{
  rhs = moveit_msgs::msg::Constraints();

  if (yaml_msg::isNode(node["name"]))
    rhs.name = node["name"].as<std::string>();

  if (yaml_msg::isNode(node["joint_constraints"]))
    rhs.joint_constraints = node["joint_constraints"].as<std::vector<moveit_msgs::msg::JointConstraint>>();

  if (yaml_msg::isNode(node["position_constraints"]))
    rhs.position_constraints = node["position_constraints"].as<std::vector<moveit_msgs::msg::PositionConstraint>>();

  if (yaml_msg::isNode(node["orientation_constraints"]))
    rhs.orientation_constraints =
        node["orientation_constraints"].as<std::vector<moveit_msgs::msg::OrientationConstraint>>();

  if (yaml_msg::isNode(node["visibility_constraints"]))
    rhs.visibility_constraints =
        node["visibility_constraints"].as<std::vector<moveit_msgs::msg::VisibilityConstraint>>();

  return true;
}

Node convert<moveit_msgs::msg::JointConstraint>::encode(const moveit_msgs::msg::JointConstraint& rhs)
{
  Node node;
  node["joint_name"] = rhs.joint_name;
  node["position"] = rhs.position;

  if (rhs.tolerance_above > std::numeric_limits<double>::epsilon())
    node["tolerance_above"] = rhs.tolerance_above;

  if (rhs.tolerance_below > std::numeric_limits<double>::epsilon())
    node["tolerance_below"] = rhs.tolerance_below;

  if (rhs.weight < 1)
    node["weight"] = rhs.weight;

  return node;
}

bool convert<moveit_msgs::msg::JointConstraint>::decode(const Node& node, moveit_msgs::msg::JointConstraint& rhs)
{
  rhs.joint_name = node["joint_name"].as<std::string>();
  rhs.position = node["position"].as<double>();

  if (yaml_msg::isNode(node["tolerance_above"]))
    rhs.tolerance_above = node["tolerance_above"].as<double>();
  else
    rhs.tolerance_above = std::numeric_limits<double>::epsilon();

  if (yaml_msg::isNode(node["tolerance_below"]))
    rhs.tolerance_below = node["tolerance_below"].as<double>();
  else
    rhs.tolerance_below = std::numeric_limits<double>::epsilon();

  if (yaml_msg::isNode(node["weight"]))
    rhs.weight = node["weight"].as<double>();
  else
    rhs.weight = 1;

  return true;
}

Node convert<moveit_msgs::msg::PositionConstraint>::encode(const moveit_msgs::msg::PositionConstraint& rhs)
{
  Node node;
  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["link_name"] = rhs.link_name;

  if (!isVector3Zero(rhs.target_point_offset))
    node["target_point_offset"] = rhs.target_point_offset;

  node["constraint_region"] = rhs.constraint_region;

  if (rhs.weight < 1)
    node["weight"] = rhs.weight;

  return node;
}

bool convert<moveit_msgs::msg::PositionConstraint>::decode(const Node& node, moveit_msgs::msg::PositionConstraint& rhs)
{
  rhs = moveit_msgs::msg::PositionConstraint();
  rhs.weight = 1;

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  rhs.link_name = node["link_name"].as<std::string>();
  if (yaml_msg::isNode(node["target_point_offset"]))
    rhs.target_point_offset = node["target_point_offset"].as<geometry_msgs::msg::Vector3>();

  rhs.constraint_region = node["constraint_region"].as<moveit_msgs::msg::BoundingVolume>();
  if (yaml_msg::isNode(node["weight"]))
    rhs.weight = node["weight"].as<double>();
  else
    rhs.weight = 1;

  return true;
}

Node convert<moveit_msgs::msg::OrientationConstraint>::encode(const moveit_msgs::msg::OrientationConstraint& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["orientation"] = rhs.orientation;
  node["link_name"] = rhs.link_name;

  node["absolute_x_axis_tolerance"] = rhs.absolute_x_axis_tolerance;
  node["absolute_y_axis_tolerance"] = rhs.absolute_y_axis_tolerance;
  node["absolute_z_axis_tolerance"] = rhs.absolute_z_axis_tolerance;

  if (rhs.weight < 1)
    node["weight"] = rhs.weight;

  return node;
}

bool convert<moveit_msgs::msg::OrientationConstraint>::decode(const Node& node,
                                                              moveit_msgs::msg::OrientationConstraint& rhs)
{
  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();
  else
    rhs.header = getDefaultHeader();

  rhs.orientation = node["orientation"].as<geometry_msgs::msg::Quaternion>();
  rhs.link_name = node["link_name"].as<std::string>();

  rhs.absolute_x_axis_tolerance = node["absolute_x_axis_tolerance"].as<double>();
  rhs.absolute_y_axis_tolerance = node["absolute_y_axis_tolerance"].as<double>();
  rhs.absolute_z_axis_tolerance = node["absolute_z_axis_tolerance"].as<double>();

  if (yaml_msg::isNode(node["weight"]))
    rhs.weight = node["weight"].as<double>();
  else
    rhs.weight = 1;

  return true;
}

Node convert<moveit_msgs::msg::VisibilityConstraint>::encode(const moveit_msgs::msg::VisibilityConstraint& rhs)
{
  Node node;
  node["target_radius"] = rhs.target_radius;
  node["target_pose"] = rhs.target_pose;
  node["cone_sides"] = rhs.cone_sides;
  node["sensor_pose"] = rhs.sensor_pose;
  node["max_view_angle"] = rhs.max_view_angle;
  node["max_range_angle"] = rhs.max_range_angle;
  node["sensor_view_direction"] = rhs.sensor_view_direction;
  node["weight"] = rhs.weight;
  return node;
}

bool convert<moveit_msgs::msg::VisibilityConstraint>::decode(const Node& node,
                                                             moveit_msgs::msg::VisibilityConstraint& rhs)
{
  rhs = moveit_msgs::msg::VisibilityConstraint();

  rhs.target_radius = node["target_radius"].as<double>();
  rhs.target_pose = node["target_pose"].as<geometry_msgs::msg::PoseStamped>();
  rhs.cone_sides = node["cone_sides"].as<int>();
  rhs.sensor_pose = node["sensor_pose"].as<geometry_msgs::msg::PoseStamped>();
  rhs.max_view_angle = node["max_view_angle"].as<double>();
  rhs.max_range_angle = node["max_range_angle"].as<double>();
  rhs.sensor_view_direction = node["sensor_view_direction"].as<int>();
  rhs.weight = node["weight"].as<double>();

  return true;
}

Node convert<moveit_msgs::msg::BoundingVolume>::encode(const moveit_msgs::msg::BoundingVolume& rhs)
{
  Node node;

  if (!rhs.primitives.empty())
  {
    node["primitives"] = rhs.primitives;
    node["primitive_poses"] = rhs.primitive_poses;
  }

  if (!rhs.meshes.empty())
  {
    node["meshes"] = rhs.meshes;
    node["mesh_poses"] = rhs.mesh_poses;
  }

  return node;
}

bool convert<moveit_msgs::msg::BoundingVolume>::decode(const Node& node, moveit_msgs::msg::BoundingVolume& rhs)
{
  rhs = moveit_msgs::msg::BoundingVolume();

  if (yaml_msg::isNode(node["primitives"]))
  {
    rhs.primitives = node["primitives"].as<std::vector<shape_msgs::msg::SolidPrimitive>>();
    rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::msg::Pose>>();
  }

  if (yaml_msg::isNode(node["meshes"]))
  {
    rhs.meshes = node["meshes"].as<std::vector<shape_msgs::msg::Mesh>>();
    rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::msg::Pose>>();
  }

  return true;
}

Node convert<moveit_msgs::msg::TrajectoryConstraints>::encode(const moveit_msgs::msg::TrajectoryConstraints& rhs)
{
  Node node;
  node["constraints"] = rhs.constraints;
  return node;
}

bool convert<moveit_msgs::msg::TrajectoryConstraints>::decode(const Node& node,
                                                              moveit_msgs::msg::TrajectoryConstraints& rhs)
{
  rhs.constraints = node["constraints"].as<std::vector<moveit_msgs::msg::Constraints>>();
  return true;
}

Node convert<moveit_msgs::msg::MotionPlanRequest>::encode(const moveit_msgs::msg::MotionPlanRequest& rhs)
{
  Node node;

  if (!(isHeaderEmpty(rhs.workspace_parameters.header) && isVector3Zero(rhs.workspace_parameters.min_corner) &&
        isVector3Zero(rhs.workspace_parameters.max_corner)))
    node["workspace_parameters"] = rhs.workspace_parameters;

  node["start_state"] = rhs.start_state;

  if (!rhs.goal_constraints.empty())
    node["goal_constraints"] = rhs.goal_constraints;

  if (!isConstraintEmpty(rhs.path_constraints))
    node["path_constraints"] = rhs.path_constraints;

  if (!rhs.trajectory_constraints.constraints.empty())
    node["trajectory_constraints"] = rhs.trajectory_constraints;

  if (!rhs.planner_id.empty())
    node["planner_id"] = rhs.planner_id;

  if (!rhs.group_name.empty())
    node["group_name"] = rhs.group_name;

  if (rhs.num_planning_attempts != 0)
    node["num_planning_attempts"] = rhs.num_planning_attempts;

  if (rhs.allowed_planning_time != 0)
    node["allowed_planning_time"] = rhs.allowed_planning_time;

  if (rhs.max_velocity_scaling_factor < 1)
    node["max_velocity_scaling_factor"] = rhs.max_velocity_scaling_factor;

  if (rhs.max_acceleration_scaling_factor < 1)
    node["max_acceleration_scaling_factor"] = rhs.max_acceleration_scaling_factor;

  return node;
}

bool convert<moveit_msgs::msg::MotionPlanRequest>::decode(const Node& node, moveit_msgs::msg::MotionPlanRequest& rhs)
{
  rhs = moveit_msgs::msg::MotionPlanRequest();

  if (yaml_msg::isNode(node["workspace_parameters"]))
    rhs.workspace_parameters = node["workspace_parameters"].as<moveit_msgs::msg::WorkspaceParameters>();

  if (yaml_msg::isNode(node["start_state"]))
    rhs.start_state = node["start_state"].as<moveit_msgs::msg::RobotState>();

  if (yaml_msg::isNode(node["goal_constraints"]))
    rhs.goal_constraints = node["goal_constraints"].as<std::vector<moveit_msgs::msg::Constraints>>();

  if (yaml_msg::isNode(node["path_constraints"]))
    rhs.path_constraints = node["path_constraints"].as<moveit_msgs::msg::Constraints>();

  if (yaml_msg::isNode(node["trajectory_constraints"]))
    rhs.trajectory_constraints = node["trajectory_constraints"].as<moveit_msgs::msg::TrajectoryConstraints>();

  if (yaml_msg::isNode(node["planner_id"]))
    rhs.planner_id = node["planner_id"].as<std::string>();

  if (yaml_msg::isNode(node["group_name"]))
    rhs.group_name = node["group_name"].as<std::string>();

  if (yaml_msg::isNode(node["num_planning_attempts"]))
    rhs.num_planning_attempts = node["num_planning_attempts"].as<int>();
  else
    rhs.num_planning_attempts = 0;

  if (yaml_msg::isNode(node["allowed_planning_time"]))
    rhs.allowed_planning_time = node["allowed_planning_time"].as<double>();
  else
    rhs.allowed_planning_time = 0;

  if (yaml_msg::isNode(node["max_velocity_scaling_factor"]))
    rhs.max_velocity_scaling_factor = node["max_velocity_scaling_factor"].as<double>();
  else
    rhs.max_velocity_scaling_factor = 1;

  if (yaml_msg::isNode(node["max_acceleration_scaling_factor"]))
    rhs.max_acceleration_scaling_factor = node["max_acceleration_scaling_factor"].as<double>();
  else
    rhs.max_acceleration_scaling_factor = 1;

  return true;
}

Node convert<trajectory_msgs::msg::MultiDOFJointTrajectory>::encode(
    const trajectory_msgs::msg::MultiDOFJointTrajectory& rhs)
{
  Node node;

  if (!isHeaderEmpty(rhs.header))
    node["header"] = rhs.header;

  node["joint_names"] = rhs.joint_names;
  node["points"] = rhs.points;

  return node;
}

bool convert<trajectory_msgs::msg::MultiDOFJointTrajectory>::decode(const Node& node,
                                                                    trajectory_msgs::msg::MultiDOFJointTrajectory& rhs)
{
  rhs = trajectory_msgs::msg::MultiDOFJointTrajectory();

  if (yaml_msg::isNode(node["header"]))
    rhs.header = node["header"].as<std_msgs::msg::Header>();

  if (yaml_msg::isNode(node["joint_names"]))
    rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

  if (yaml_msg::isNode(node["points"]))
    rhs.points = node["points"].as<std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>>();

  return true;
}

Node convert<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::encode(
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& rhs)
{
  Node node;

  if (!rhs.transforms.empty())
    node["transforms"] = rhs.transforms;

  if (!rhs.velocities.empty())
    node["velocities"] = rhs.velocities;

  if (!rhs.accelerations.empty())
    node["accelerations"] = rhs.accelerations;

  node["time_from_start"] = rhs.time_from_start.sec + rhs.time_from_start.nanosec * 1e9;

  return node;
}

bool convert<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::decode(
    const Node& node, trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& rhs)
{
  rhs = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint();

  if (yaml_msg::isNode(node["transforms"]))
    rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::msg::Transform>>();

  if (yaml_msg::isNode(node["velocities"]))
    rhs.velocities = node["velocities"].as<std::vector<geometry_msgs::msg::Twist>>();

  if (yaml_msg::isNode(node["accelerations"]))
    rhs.accelerations = node["accelerations"].as<std::vector<geometry_msgs::msg::Twist>>();

  double duration = node["time_from_start"].as<double>();
  double sec = std::floor(duration);
  unsigned long nanosec = (duration - sec) * 1e9;
  rhs.time_from_start = builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec((int)sec).nanosec(nanosec);

  return true;
}

Node convert<moveit_msgs::msg::RobotTrajectory>::encode(const moveit_msgs::msg::RobotTrajectory& rhs)
{
  Node node;

  if (!rhs.joint_trajectory.points.empty())
    node["joint_trajectory"] = rhs.joint_trajectory;

  if (!rhs.multi_dof_joint_trajectory.points.empty())
    node["multi_dof_joint_trajectory"] = rhs.multi_dof_joint_trajectory;

  return node;
}

bool convert<moveit_msgs::msg::RobotTrajectory>::decode(const Node& node, moveit_msgs::msg::RobotTrajectory& rhs)
{
  rhs = moveit_msgs::msg::RobotTrajectory();

  if (yaml_msg::isNode(node["joint_trajectory"]))
    rhs.joint_trajectory = node["joint_trajectory"].as<trajectory_msgs::msg::JointTrajectory>();

  if (yaml_msg::isNode(node["multi_dof_joint_trajectory"]))
    rhs.multi_dof_joint_trajectory =
        node["multi_dof_joint_trajectory"].as<trajectory_msgs::msg::MultiDOFJointTrajectory>();

  return true;
}
}  // namespace YAML

namespace yaml_msg
{
bool isNode(const YAML::Node& node)
{
  try
  {
    bool r = node.IsDefined() and not node.IsNull();
    if (r)
      try
      {
        r = node.as<std::string>() != "~";
      }
      catch (std::exception& e)
      {
      }

    return r;
  }
  catch (YAML::InvalidNode& e)
  {
    return false;
  }
}

moveit_msgs::msg::RobotState robotStateFromNode(const YAML::Node& node)
{
  return node.as<moveit_msgs::msg::RobotState>();
}

YAML::Node toNode(const geometry_msgs::msg::Pose& msg)
{
  YAML::Node node;
  node = msg;
  return node;
}

geometry_msgs::msg::Pose poseFromNode(const YAML::Node& node)
{
  return node.as<geometry_msgs::msg::Pose>();
}

YAML::Node toNode(const moveit_msgs::msg::PlanningScene& msg)
{
  YAML::Node node;
  node = msg;
  return node;
}

YAML::Node toNode(const moveit_msgs::msg::MotionPlanRequest& msg)
{
  YAML::Node node;
  node = msg;
  return node;
}

YAML::Node toNode(const moveit_msgs::msg::RobotTrajectory& msg)
{
  YAML::Node node;
  node = msg;
  return node;
}

YAML::Node toNode(const moveit_msgs::msg::RobotState& msg)
{
  YAML::Node node;
  node = msg;
  return node;
}

std::pair<bool, YAML::Node> loadFileToYAML(const std::string& full_path)
{
  YAML::Node file;
  if (full_path.empty())
    return std::make_pair(false, file);

  try
  {
    return std::make_pair(true, YAML::LoadFile(full_path));
  }
  catch (std::exception& e)
  {
    return std::make_pair(false, file);
  }
}

/** \brief Load a message (or YAML convertible object) from a file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load message from.
 *  \tparam T Type of the message.
 *  \return True on success, false on failure.
 */
template <typename T>
bool YAMLFileToMessage(T& msg, const std::string& file)
{
  const auto& result = loadFileToYAML(file);
  if (result.first)
    msg = result.second.as<T>();

  return result.first;
}

bool fromYAMLFile(moveit_msgs::msg::PlanningScene& msg, const std::string& file)
{
  return YAMLFileToMessage(msg, file);
}

bool fromYAMLFile(moveit_msgs::msg::MotionPlanRequest& msg, const std::string& file)
{
  return YAMLFileToMessage(msg, file);
}

bool fromYAMLFile(moveit_msgs::msg::RobotTrajectory& msg, const std::string& file)
{
  return YAMLFileToMessage(msg, file);
}

bool fromYAMLFile(moveit_msgs::msg::RobotState& msg, const std::string& file)
{
  return YAMLFileToMessage(msg, file);
}
}  // namespace yaml_msg
