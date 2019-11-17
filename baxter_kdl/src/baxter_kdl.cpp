#include <baxter_kdl/baxter_kdl.h>

namespace B_KDL
{

using namespace KDL;

BaxterDynamics::BaxterDynamics(const std::string &limb_name)
{
  m_limb_name = limb_name;
  if (m_limb_name != "right" && m_limb_name != "left")
  {
    ROS_ERROR("Recieve a wrong limb name");
    // throw exception
  }

  m_spring_effort = -30;
  double m_default_threshold_[] = {10.0, 10.0, 5.0, 4.5, 2.0, 2.0, 1.0}; // collision threshold
  m_is_collision = false;

  ros::NodeHandle node;
  ros::Rate loop_rate(10);

  // Create a subscribers
  m_joint_states_sub = node.subscribe("/robot/joint_states", 100, &BaxterDynamics::_JointStateCallback, this);
  std::string arm_states_topic_name = "/robot/limb/" + m_limb_name + "/gravity_compensation_torques";
  m_arm_states_sub = node.subscribe(arm_states_topic_name.c_str(), 100, &BaxterDynamics::_ArmStateCallback, this);

  // Read the URDF from param sever
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromParam("robot_description", kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }

  // Get chain from kdl tree
  std::string base_link = "base";
  std::string tip_link = m_limb_name + "_gripper"; // _gripper frame
  kdl_tree.getChain(base_link, tip_link, m_arm_chain);
  //Get number of Joints
  m_num_jnts = m_arm_chain.getNrOfJoints();

  // Some initializations
  m_new_time = ros::Time::now();
  m_q_current.resize(m_num_jnts);
  m_q_dot_current.resize(m_num_jnts);
  m_q_ddot_current.resize(m_num_jnts);
  m_q_effort_current.resize(m_num_jnts);
  m_external_torque.resize(m_num_jnts);
  m_default_threshold.resize(m_num_jnts);
  m_threshold.resize(m_num_jnts);

  for (size_t i = 0; i < m_num_jnts; i++)
  {
    m_default_threshold[i] = m_default_threshold_[i];
  }
  m_threshold = m_default_threshold;

  m_last_q_current.resize(m_num_jnts);
  m_last_q_dot_current.resize(m_num_jnts);

  m_is_joint_state_ready = false;
  // Wait for first message from joint_state subscriber arrives
  while (ros::ok() && !m_is_joint_state_ready)
  {
    ros::spinOnce();
  }
}

bool BaxterDynamics::SetCollisionThreshold(const std::vector<double> &threshold)
{
  if (threshold.size() != m_num_jnts)
  {
    ROS_ERROR("Invalid input");
    return false;
  }
  for (size_t i = 0; i < m_num_jnts; ++i)
  {
    if (threshold[i] < 0)
    {
      ROS_ERROR("Invalid input");
      return false;
    }
  }
  m_threshold = threshold;
  return true;
}

bool BaxterDynamics::SetDefaultCollisionThreshold()
{
  return SetCollisionThreshold(m_default_threshold);
}

bool BaxterDynamics::ForwardKinematics(const std::vector<double> &joint_values, std::vector<double> &posture)
{
  if (joint_values.size() != m_num_jnts)
  {
    ROS_ERROR("Input joint_values has a wrong size");
    return false;
  }
  posture.resize(7); //  x y z ox oy oz ow

  //Create Joint Array for calculation
  KDL::JntArray joint_positions = JntArray(m_num_jnts);

  for (size_t i = 0; i < m_num_jnts; i++)
  {
    joint_positions(i) = joint_values[i];
  }

  //Create the frame that will contain the results
  KDL::Frame end_frame;

  ChainFkSolverPos_recursive fksolver(m_arm_chain);

  if (fksolver.JntToCart(joint_positions, end_frame) != 0)
  {
    ROS_ERROR("Cannot compute forward kinematics");
    return false;
  }
  KDL::Vector position;
  KDL::Rotation rotation = Rotation(end_frame.M);
  position = end_frame.p;
  double ox, oy, oz, ow;
  rotation.GetQuaternion(ox, oy, oz, ow);

  posture[0] = position(0);
  posture[1] = position(1);
  posture[2] = position(2);
  posture[3] = ox;
  posture[4] = oy;
  posture[5] = oz;
  posture[6] = ow;

  return true;
}

bool BaxterDynamics::ForwardKinematics(std::vector<double> &posture)
{
  return ForwardKinematics(m_q_current, posture);
}

bool BaxterDynamics::InverseKinematics(const std::vector<double> &position, std::vector<double> &joint_values)
{
  std::vector<double> orientation;
  orientation.clear(); // zero size orientation
  return InverseKinematics(position, orientation, joint_values);
}

bool BaxterDynamics::InverseKinematics(const std::vector<double> &position, const std::vector<double> &orientation, std::vector<double> &joint_values)
{
  return InverseKinematics(position, orientation, m_q_current, joint_values);
}

bool BaxterDynamics::InverseKinematics(const std::vector<double> &position, const std::vector<double> &orientation, const std::vector<double> &seed, std::vector<double> &joint_values)
{
  if (position.size() != 3)
  {
    ROS_ERROR("Invalid position size");
    return false;
  }
  if (orientation.size() != 4 && orientation.size() != 0)
  {
    ROS_ERROR("Invalid orientation size");
    return false;
  }
  if (seed.size() != m_num_jnts)
  {
    ROS_ERROR("Invalid seed size");
    return false;
  }
  joint_values.resize(m_num_jnts);

  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(m_arm_chain);
  ChainIkSolverVel_pinv iksolver_v = ChainIkSolverVel_pinv(m_arm_chain);
  ChainIkSolverPos_NR iksolver_p = ChainIkSolverPos_NR(m_arm_chain, fksolver, iksolver_v);
  KDL::Vector pos = Vector(position[0], position[1], position[2]);
  KDL::Rotation rot = Rotation();
  if (orientation.size() != 0)
  {
    rot = rot.Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
  }

  KDL::JntArray seed_array = JntArray(m_num_jnts);

  for (size_t i = 0; i < m_num_jnts; i++)
  {
    seed_array(i) = seed[i];
  }

  //Make IK Call
  KDL::Frame goal_pose;
  if (orientation.size() != 0)
  {
    goal_pose = Frame(rot, pos);
  }
  else
  {
    goal_pose = Frame(pos);
  }
  KDL::JntArray result_angles = JntArray(m_num_jnts);

  if (iksolver_p.CartToJnt(seed_array, goal_pose, result_angles) != KDL::SolverI::E_NOERROR)
  {
    ROS_ERROR("Cannot compute inverse kinematics");
    return false;
  }

  for (size_t i = 0; i < m_num_jnts; i++)
  {
    joint_values[i] = result_angles(i);
  }
  return true;
}

bool BaxterDynamics::GravityEffort(std::vector<double> &effort)
{
  // current gravity effort
  return GravityEffort(m_q_current, effort);
}

bool BaxterDynamics::GravityEffort(const std::vector<double> &joint_values, std::vector<double> &effort)
{
  if (joint_values.size() != m_num_jnts)
  {
    return false;
  }
  effort.resize(m_num_jnts);

  KDL::ChainDynParam dyn = KDL::ChainDynParam(m_arm_chain, KDL::Vector(0, 0, -9.81));
  //Create Joint Array for calculation
  KDL::JntArray joint_positions = JntArray(m_num_jnts);

  for (size_t i = 0; i < m_num_jnts; i++)
  {
    joint_positions(i) = joint_values[i];
  }

  KDL::JntArray gravity = JntArray(m_num_jnts); // gravity array
  if (dyn.JntToGravity(joint_positions, gravity) != KDL::SolverI::E_NOERROR)
  {
    ROS_ERROR("Cannot compute gravity torque");
    return false;
  }

  // Modifying the output array
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    effort[i] = gravity(i);
  }
  effort[1] -= m_spring_effort; // subtract spring effort from s1 joint
  return true;
}

bool BaxterDynamics::InverseDynamics(std::vector<double> &effort)
{
  //TODO: calculate m_q_ddot_current
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    m_q_ddot_current[i] = 0; // (last_q_dot - q_dot)/dt is too bad.
  }

  return InverseDynamics(m_q_current, m_q_dot_current, m_q_ddot_current, effort);
}

bool BaxterDynamics::InverseDynamics(const std::vector<double> &joint_values, const std::vector<double> &joint_velocities, const std::vector<double> &joint_accelerations, std::vector<double> &effort)
{
  if (joint_values.size() != m_num_jnts)
  {
    ROS_ERROR("Invalid joint_values size");
    return false;
  }
  if (joint_velocities.size() != m_num_jnts)
  {
    ROS_ERROR("Invalid joint_velocities size");
    return false;
  }
  if (joint_accelerations.size() != m_num_jnts)
  {
    ROS_ERROR("Invalid joint_accelerations size");
    return false;
  }

  effort.resize(m_num_jnts);

  // inverse dynamics solver: recursive newton euler solver
  KDL::ChainIdSolver_RNE idsolver(m_arm_chain, KDL::Vector(0, 0, -9.81));

  KDL::JntArray q = JntArray(m_num_jnts);
  KDL::JntArray q_dot = JntArray(m_num_jnts);
  KDL::JntArray q_dotdot = JntArray(m_num_jnts);
  std::vector<Wrench> wrenchnull;
  KDL::JntArray torques = JntArray(m_num_jnts);

  for (size_t i = 0; i < m_arm_chain.getNrOfSegments(); i++)
  {
    wrenchnull.push_back(KDL::Wrench());
  }

  for (size_t i = 0; i < m_num_jnts; i++)
  {
    q(i) = joint_values[i];
    q_dot(i) = joint_velocities[i];
    q_dotdot(i) = joint_accelerations[i];
  }

  if (idsolver.CartToJnt(q, q_dot, q_dotdot, wrenchnull, torques) != KDL::SolverI::E_NOERROR)
  {
    ROS_ERROR("Cannot compute inverse dynamics");
    return false;
  }
  //Modifying the output array
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    effort[i] = torques(i);
  }
  effort[1] -= m_spring_effort; // subtract spring effort from e1 joint
  return true;
}

void BaxterDynamics::_JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  //setting joints in the right order to feed the jacobian
  //JointState order -> e0, e1, s0, s1, w0, w1, w2
  //desired order -> s0, s1, e0, e1, w0, w1, w2

  if (msg->name.size() == 1)
  {
    // filter out gripper joint_states
    return;
  }

  m_old_time = m_new_time;
  m_new_time = msg->header.stamp;
  m_time_step = m_new_time - m_old_time;
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    m_last_q_current[i] = m_q_current[i];
    m_last_q_dot_current[i] = m_q_dot_current[i];
  }

  int t = 0;
  if (m_limb_name == "right")
  {
    t = 7;
  }
  for (size_t i = 0; i < 2; i++)
  {
    // s0 and s1
    m_q_current[i] = (double)(msg->position[i + 4 + t]);
    m_q_dot_current[i] = (double)(msg->velocity[i + 4 + t]);
    m_q_effort_current[i] = (double)(msg->effort[i + 4 + t]);
  }

  for (size_t i = 2; i < 4; i++)
  {
    // e0 and e1
    m_q_current[i] = (double)(msg->position[i + t]);
    m_q_dot_current[i] = (double)(msg->velocity[i + t]);
    m_q_effort_current[i] = (double)(msg->effort[i + t]);
  }

  for (size_t i = 4; i < 7; i++)
  {
    // w0, w1, w2
    m_q_current[i] = (double)(msg->position[i + 2 + t]);
    m_q_dot_current[i] = (double)(msg->velocity[i + 2 + t]);
    m_q_effort_current[i] = (double)(msg->effort[i + 2 + t]);
  }

  // TODO: compute joint accelerations with Kalman Filter.
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    // dirty accelerations
    m_q_ddot_current[i] = (m_q_dot_current[i] - m_last_q_dot_current[i]) / m_time_step.toSec();
  }

  m_is_joint_state_ready = true;
  _ComputeExternalTorque(); // update external torque
  _UpdateCollisionState();  // update collision state
}

void BaxterDynamics::_ArmStateCallback(const baxter_core_msgs::SEAJointState::ConstPtr &msg)
{
  // this arm_state is slower than joint_states, but it can supply the hysteresis_model_effort and crosstalk_model_effort: (spring effort for s1 joint)
  m_spring_effort = msg->hysteresis_model_effort[1] - msg->crosstalk_model_effort[1];
}

void BaxterDynamics::_ComputeExternalTorque()
{
  std::vector<double> id_result; // computed torque from inverse dynamics
  id_result.clear();

  InverseDynamics(id_result);
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    m_external_torque[i] = m_q_effort_current[i] - id_result[i];
  }
}

bool BaxterDynamics::_UpdateCollisionState()
{
  m_is_collision = false;
  for (size_t i = 0; i < m_num_jnts; i++)
  {
    if (std::abs(m_external_torque[i]) > m_threshold[i])
    {
      m_is_collision = true;
    }
  }
  return m_is_collision;
}

} // namespace B_KDL
