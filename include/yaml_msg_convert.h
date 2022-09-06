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
// See https://github.com/KavrakiLab/robowflex/blob/master/robowflex_library/include/robowflex_library/yaml.h

#ifndef YAML_CONVERT_
#define YAML_CONVERT_

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/plane.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/msg/octomap_with_pose.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <object_recognition_msgs/msg/object_type.hpp>

#include <moveit_msgs/msg/link_padding.hpp>
#include <moveit_msgs/msg/link_scale.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/visibility_constraint.hpp>
#include <moveit_msgs/msg/trajectory_constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

//#include <moveit_msgs/msg/move_group_goal.hpp>
//#include <moveit_msgs/msg/move_group_result.hpp>

#include <yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<moveit_msgs::msg::PlanningScene>
{
  static Node encode(const moveit_msgs::msg::PlanningScene& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::PlanningScene& rhs);
};

template <>
struct convert<moveit_msgs::msg::RobotState>
{
  static Node encode(const moveit_msgs::msg::RobotState& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::RobotState& rhs);
};

template <>
struct convert<geometry_msgs::msg::TransformStamped>
{
  static Node encode(const geometry_msgs::msg::TransformStamped& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::TransformStamped& rhs);
};

template <>
struct convert<std_msgs::msg::Header>
{
  static Node encode(const std_msgs::msg::Header& rhs);
  static bool decode(const Node& node, std_msgs::msg::Header& rhs);
};

template <>
struct convert<geometry_msgs::msg::Pose>
{
  static Node encode(const geometry_msgs::msg::Pose& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Pose& rhs);
};

template <>
struct convert<geometry_msgs::msg::PoseStamped>
{
  static Node encode(const geometry_msgs::msg::PoseStamped& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::PoseStamped& rhs);
};

template <>
struct convert<geometry_msgs::msg::Transform>
{
  static Node encode(const geometry_msgs::msg::Transform& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Transform& rhs);
};

template <>
struct convert<geometry_msgs::msg::Vector3>
{
  static Node encode(const geometry_msgs::msg::Vector3& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Vector3& rhs);
};

template <>
struct convert<geometry_msgs::msg::Point>
{
  static Node encode(const geometry_msgs::msg::Point& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Point& rhs);
};

template <>
struct convert<geometry_msgs::msg::Quaternion>
{
  static Node encode(const geometry_msgs::msg::Quaternion& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Quaternion& rhs);
};

template <>
struct convert<geometry_msgs::msg::Twist>
{
  static Node encode(const geometry_msgs::msg::Twist& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Twist& rhs);
};

template <>
struct convert<geometry_msgs::msg::Wrench>
{
  static Node encode(const geometry_msgs::msg::Wrench& rhs);
  static bool decode(const Node& node, geometry_msgs::msg::Wrench& rhs);
};

template <>
struct convert<sensor_msgs::msg::JointState>
{
  static Node encode(const sensor_msgs::msg::JointState& rhs);
  static bool decode(const Node& node, sensor_msgs::msg::JointState& rhs);
};

template <>
struct convert<sensor_msgs::msg::MultiDOFJointState>
{
  static Node encode(const sensor_msgs::msg::MultiDOFJointState& rhs);
  static bool decode(const Node& node, sensor_msgs::msg::MultiDOFJointState& rhs);
};

template <>
struct convert<moveit_msgs::msg::AttachedCollisionObject>
{
  static Node encode(const moveit_msgs::msg::AttachedCollisionObject& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::AttachedCollisionObject& rhs);
};

template <>
struct convert<trajectory_msgs::msg::JointTrajectory>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectory& rhs);
  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectory& rhs);
};

template <>
struct convert<trajectory_msgs::msg::JointTrajectoryPoint>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectoryPoint& rhs);
  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectoryPoint& rhs);
};

template <>
struct convert<moveit_msgs::msg::CollisionObject>
{
  static Node encode(const moveit_msgs::msg::CollisionObject& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::CollisionObject& rhs);
};

template <>
struct convert<object_recognition_msgs::msg::ObjectType>
{
  static Node encode(const object_recognition_msgs::msg::ObjectType& rhs);
  static bool decode(const Node& node, object_recognition_msgs::msg::ObjectType& rhs);
};

template <>
struct convert<moveit_msgs::msg::LinkPadding>
{
  static Node encode(const moveit_msgs::msg::LinkPadding& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::LinkPadding& rhs);
};

template <>
struct convert<moveit_msgs::msg::LinkScale>
{
  static Node encode(const moveit_msgs::msg::LinkScale& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::LinkScale& rhs);
};

template <>
struct convert<moveit_msgs::msg::AllowedCollisionMatrix>
{
  static Node encode(const moveit_msgs::msg::AllowedCollisionMatrix& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::AllowedCollisionMatrix& rhs);
};

template <>
struct convert<moveit_msgs::msg::AllowedCollisionEntry>
{
  static Node encode(const moveit_msgs::msg::AllowedCollisionEntry& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::AllowedCollisionEntry& rhs);
};

template <>
struct convert<moveit_msgs::msg::PlanningSceneWorld>
{
  static Node encode(const moveit_msgs::msg::PlanningSceneWorld& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::PlanningSceneWorld& rhs);
};

template <>
struct convert<moveit_msgs::msg::ObjectColor>
{
  static Node encode(const moveit_msgs::msg::ObjectColor& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::ObjectColor& rhs);
};

template <>
struct convert<std_msgs::msg::ColorRGBA>
{
  static Node encode(const std_msgs::msg::ColorRGBA& rhs);
  static bool decode(const Node& node, std_msgs::msg::ColorRGBA& rhs);
};

template <>
struct convert<octomap_msgs::msg::Octomap>
{
  static Node encode(const octomap_msgs::msg::Octomap& rhs);
  static bool decode(const Node& node, octomap_msgs::msg::Octomap& rhs);
};

template <>
struct convert<octomap_msgs::msg::OctomapWithPose>
{
  static Node encode(const octomap_msgs::msg::OctomapWithPose& rhs);
  static bool decode(const Node& node, octomap_msgs::msg::OctomapWithPose& rhs);
};

template <>
struct convert<rclcpp::Time>
{
  static Node encode(const rclcpp::Time& rhs);
  static bool decode(const Node& node, rclcpp::Time& rhs);
};

template <>
struct convert<rclcpp::Duration>
{
  static Node encode(const rclcpp::Duration& rhs);
  static bool decode(const Node& node, rclcpp::Duration& rhs);
};

template <>
struct convert<shape_msgs::msg::SolidPrimitive>
{
  static Node encode(const shape_msgs::msg::SolidPrimitive& rhs);
  static bool decode(const Node& node, shape_msgs::msg::SolidPrimitive& rhs);
};

template <>
struct convert<shape_msgs::msg::Mesh>
{
  static Node encode(const shape_msgs::msg::Mesh& rhs);
  static bool decode(const Node& node, shape_msgs::msg::Mesh& rhs);
};

template <>
struct convert<shape_msgs::msg::MeshTriangle>
{
  static Node encode(const shape_msgs::msg::MeshTriangle& rhs);
  static bool decode(const Node& node, shape_msgs::msg::MeshTriangle& rhs);
};

template <>
struct convert<shape_msgs::msg::Plane>
{
  static Node encode(const shape_msgs::msg::Plane& rhs);
  static bool decode(const Node& node, shape_msgs::msg::Plane& rhs);
};

template <>
struct convert<moveit_msgs::msg::WorkspaceParameters>
{
  static Node encode(const moveit_msgs::msg::WorkspaceParameters& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::WorkspaceParameters& rhs);
};

template <>
struct convert<moveit_msgs::msg::Constraints>
{
  static Node encode(const moveit_msgs::msg::Constraints& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::Constraints& rhs);
};

template <>
struct convert<moveit_msgs::msg::JointConstraint>
{
  static Node encode(const moveit_msgs::msg::JointConstraint& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::JointConstraint& rhs);
};

template <>
struct convert<moveit_msgs::msg::PositionConstraint>
{
  static Node encode(const moveit_msgs::msg::PositionConstraint& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::PositionConstraint& rhs);
};

template <>
struct convert<moveit_msgs::msg::OrientationConstraint>
{
  static Node encode(const moveit_msgs::msg::OrientationConstraint& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::OrientationConstraint& rhs);
};

template <>
struct convert<moveit_msgs::msg::VisibilityConstraint>
{
  static Node encode(const moveit_msgs::msg::VisibilityConstraint& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::VisibilityConstraint& rhs);
};

template <>
struct convert<moveit_msgs::msg::TrajectoryConstraints>
{
  static Node encode(const moveit_msgs::msg::TrajectoryConstraints& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::TrajectoryConstraints& rhs);
};

template <>
struct convert<moveit_msgs::msg::BoundingVolume>
{
  static Node encode(const moveit_msgs::msg::BoundingVolume& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::BoundingVolume& rhs);
};

template <>
struct convert<moveit_msgs::msg::MotionPlanRequest>
{
  static Node encode(const moveit_msgs::msg::MotionPlanRequest& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::MotionPlanRequest& rhs);
};

template <>
struct convert<trajectory_msgs::msg::MultiDOFJointTrajectory>
{
  static Node encode(const trajectory_msgs::msg::MultiDOFJointTrajectory& rhs);
  static bool decode(const Node& node, trajectory_msgs::msg::MultiDOFJointTrajectory& rhs);
};

template <>
struct convert<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>
{
  static Node encode(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& rhs);
  static bool decode(const Node& node, trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& rhs);
};

template <>
struct convert<moveit_msgs::msg::RobotTrajectory>
{
  static Node encode(const moveit_msgs::msg::RobotTrajectory& rhs);
  static bool decode(const Node& node, moveit_msgs::msg::RobotTrajectory& rhs);
};
}  // namespace YAML

namespace yaml_msg
{
/** \brief Checks if a key exists within a YAML node.
 *  \param[in] node Node to check.
 *  \return True if the node exists and is not null.
 */
bool isNode(const YAML::Node& node);

/** \brief Converts a robot state YAML to a robot_state message.
 *  \param[in] node Node to convert.
 *  \return The converted message.
 */
moveit_msgs::msg::RobotState robotStateFromNode(const YAML::Node& node);

/** \brief Converts a pose message to YAML.
 *  \param[in] msg Message to convert.
 *  \return The converted message.
 */
YAML::Node toNode(const geometry_msgs::msg::Pose& msg);

/** \brief Converts a pose YAML to a goemetry message.
 *  \param[in] node Node to convert.
 *  \return The converted message.
 */
geometry_msgs::msg::Pose poseFromNode(const YAML::Node& node);

/** \brief Converts a planning scene message to YAML.
 *  \param[in] msg Message to convert.
 *  \return The converted message.
 */
YAML::Node toNode(const moveit_msgs::msg::PlanningScene& msg);

/** \brief Converts a motion plan request to YAML.
 *  \param[in] msg Message to convert.
 *  \return The converted message.
 */
YAML::Node toNode(const moveit_msgs::msg::MotionPlanRequest& msg);

/** \brief Converts a motion plan to YAML.
 *  \param[in] msg Message to convert.
 *  \return The converted message.
 */
YAML::Node toNode(const moveit_msgs::msg::RobotTrajectory& msg);

/** \brief Converts a robot state to YAML.
 *  \param[in] msg Message to convert.
 *  \return The converted message.
 */
YAML::Node toNode(const moveit_msgs::msg::RobotState& msg);

/** \brief Loads a planning scene from a YAML file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load.
 *  \return True on success, false on failure.
 */
bool fromYAMLFile(moveit_msgs::msg::PlanningScene& msg, const std::string& file);

/** \brief Loads a motion planning request from a YAML file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load.
 *  \return True on success, false on failure.
 */
bool fromYAMLFile(moveit_msgs::msg::MotionPlanRequest& msg, const std::string& file);

/** \brief Loads a trajectory from a YAML file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load.
 *  \return True on success, false on failure.
 */
bool fromYAMLFile(moveit_msgs::msg::RobotTrajectory& msg, const std::string& file);

/** \brief Loads a robot state from a YAML file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load.
 *  \return True on success, false on failure.
 */
bool fromYAMLFile(moveit_msgs::msg::RobotState& msg, const std::string& file);
}  // namespace yaml_msg

#endif
