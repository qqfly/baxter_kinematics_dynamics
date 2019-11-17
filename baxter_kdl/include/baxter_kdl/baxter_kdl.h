// GNU Lesser General Public License
//
// Copyright (c) 2017 Qiang Qiu <qrobotics@yeah.net>
//
// BaxterDynamics is a free software, you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#pragma once

#include <stdio.h>
// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/SEAJointState.h>

// kdl
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

// using namespace KDL;

namespace B_KDL
{
class BaxterDynamics
{
public:
  /**
   * @brief Construct a new Baxter Dynamics
   * 
   * @param[in] limb_name: for baxter, limb_name = "right" or "left"
   */
  explicit BaxterDynamics(const std::string &limb_name);

  /**
   * @brief Set the Collision Threshold
   * 
   * @param[in] threshold: torque threshold for collision detection
   * @return true
   * @return false: input threshold is invalid (invalid size or value)
   */
  bool SetCollisionThreshold(const std::vector<double> &threshold);
  bool SetDefaultCollisionThreshold();

  /**
   * @brief Compute forward kinematics with given joint values
   * 
   * @param[in] joint_values: joint values
   * @param[out] posture: forward kinematics result, [x, y, z, ox, oy, oz, ow]
   * @return true 
   * @return false 
   */
  bool ForwardKinematics(const std::vector<double> &joint_values, std::vector<double> &posture);
  bool ForwardKinematics(std::vector<double> &posture);

  /**
   * @brief Compute position-only inverse kinematics with given position
   * 
   * @param[in] position: x, y, z
   * @param[out] joint_values: joint value
   * @return true 
   * @return false 
   */
  bool InverseKinematics(const std::vector<double> &position, std::vector<double> &joint_values);

  /**
   * @brief Compute 6d transform IK with given position and orientation, search from current joint_values
   * 
   * @param[in] position: x, y, z
   * @param[in] orientation: ox, oy, oz, ow
   * @param[out] joint_values: joint values
   * @return true
   * @return false 
   */
  bool InverseKinematics(const std::vector<double> &position, const std::vector<double> &orientation, std::vector<double> &joint_values);

  /**
   * @brief Compute 6d transform IK with given position and orientation, search from given joint-seed
   * 
   * @param[in] position: x, y, z
   * @param[in] orientation: ox, oy, oz, ow
   * @param[in] seed: joint seed values
   * @param[out] joint_values: joint values
   * @return true 
   * @return false 
   */
  bool InverseKinematics(const std::vector<double> &position, const std::vector<double> &orientation, const std::vector<double> &seed, std::vector<double> &joint_values);

  /**
   * @brief Compute gravity force in given configuration
   * 
   * @param[in] joint_values
   * @param[out] effort 
   * @return true 
   * @return false 
   */
  bool GravityEffort(const std::vector<double> &joint_values, std::vector<double> &effort);
  bool GravityEffort(std::vector<double> &effort); // with current joint states

  /**
   * @brief Compute inverse dynamics
   * 
   * @param[in] joint_values 
   * @param[in] joint_velocities 
   * @param[in] joint_accelerations 
   * @param[out] effort 
   * @return true 
   * @return false 
   */
  bool InverseDynamics(const std::vector<double> &joint_values, const std::vector<double> &joint_velocities, const std::vector<double> &joint_accelerations, std::vector<double> &effort);
  bool InverseDynamics(std::vector<double> &effort); // with current joint states

public:
  KDL::Chain m_arm_chain;
  std::vector<double> m_external_torque; // external torque f_real - f_id;
  bool m_is_collision;                   // robot collision state

private:
  bool m_is_joint_state_ready;
  double m_spring_effort;                  // spring effort for s1 joint
  std::vector<double> m_threshold;         // threshold value for collision detection
  std::vector<double> m_default_threshold; // default threshold value for collision detection
  size_t m_num_jnts;                       // number of joints
  std::string m_limb_name;
  std::vector<double> m_q_current;          // joint values
  std::vector<double> m_q_dot_current;      // joint velocity values
  std::vector<double> m_q_ddot_current;     // joint accelerations
  std::vector<double> m_q_effort_current;   // real joint effort
  std::vector<double> m_last_q_current;     // last joint positions
  std::vector<double> m_last_q_dot_current; // last joint velocity
  ros::Time m_new_time;
  ros::Time m_old_time;
  ros::Duration m_time_step;
  ros::Subscriber m_joint_states_sub; // joint states subscriber
  ros::Subscriber m_arm_states_sub;   // arm state subscriber

private:
  void _JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void _ArmStateCallback(const baxter_core_msgs::SEAJointState::ConstPtr &msg);
  //void _CalculateAccelerations()；    // compute joint accelerations with Kalman Filter
  void _ComputeExternalTorque(); // compute external torque
  bool _UpdateCollisionState();
};

} // namespace B_KDL
