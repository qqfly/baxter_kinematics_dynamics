// GNU Lesser General Public License
// 
// Copyright (c) 2017 Qiang Qiu <qrobotics@yeah.net>
// 
// baxter_kdl is a free software, you can redistribute it and/or modify
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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <baxter_core_msgs/SEAJointState.h>

#ifndef K_DYNAMICS_H
#define K_DYNAMICS_H

using namespace std;
using namespace KDL;

namespace B_KDL {
class baxter_kdl
{
  public:
    KDL::Chain arm_chain;
    vector<double> q_current;	                // joint values
    vector<double> q_dot_current;	            // joint velocity values
    vector<double> q_ddot_current;            // joint accelerations
    vector<double> q_effort_current;          // real joint effort
    vector<double> external_torque;           // external torque f_real - f_id;
    bool is_collision;                        // robot collision state
    baxter_kdl(std::string limb);             // For baxter, limb = "right" / "left"
    void set_collision_threshold(vector<double> threshold_);                          // set threshold value for collision detection
    void set_default_collision_threshold();                                           // set threshold value for collision detection as default
    bool forward_kinematics(vector<double> &result);                                  // FK with current joint states
    bool forward_kinematics(vector<double> &result, vector<double> joint_values);     // FK with given joint states
    bool inverse_kinematics(vector<double> &result, vector<double> position);         // position only IK
    bool inverse_kinematics(vector<double> &result, vector<double> position, vector<double> orientation);                       // 6d IK with current seed
    bool inverse_kinematics(vector<double> &result, vector<double> position, vector<double> orientation, vector<double> seed);  // 6d IK with given seed
    bool gravity_effort(vector<double> &result);                                      // with current joint states
    bool gravity_effort(vector<double> &result, vector<double> joint_values);
    bool inverse_dynamics(vector<double> &result);                                    // with current joint states
    bool inverse_dynamics(vector<double> &result, vector<double> joint_values, vector<double> joint_velocities, vector<double> joint_accelerations);

  private:
    bool is_joint_state_ready;
    double spring_effort;                 // spring effort for s1 joint
    vector<double> threshold;             // threshold value for collision detection
    vector<double> default_threshold;     // default threshold value for collision detection
    int num_jnts;                         // number of joints
    std::string limb;
    KDL::Tree kdl_tree;
    vector<double> last_q_current;        // last joint positions
    vector<double> last_q_dot_current;    // last joint velocity
    ros::Time new_time;
    ros::Time old_time;
    ros::Duration time_step;
    ros::Subscriber joint_states_sub;     // joint states subscriber
    ros::Subscriber arm_states_sub;       // arm state subscriber
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void arm_state_callback(const baxter_core_msgs::SEAJointState::ConstPtr& msg);
    //void calculate_accelerations()；    // compute joint accelerations with Kalman Filter
    void compute_external_torque();       // compute external torque
    bool collision_state();
};

}

#endif
