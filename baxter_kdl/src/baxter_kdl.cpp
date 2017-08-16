#include <baxter_kdl.h>

namespace B_KDL {

using namespace std;
using namespace KDL;

baxter_kdl::baxter_kdl(std::string _limb)
{
  limb = _limb;
  if(limb != "right" && limb != "left")
  {
    ROS_ERROR("Recieve a wrong limb name");
    //break();
  }

  is_joint_state_ready = 1;
  spring_effort = -30;
  double default_threshold_[] = {10.0, 10.0, 5.0, 4.5, 2.0, 2.0, 1.0};  // collision threshold
  is_collision = false;

  ros::NodeHandle node;
  ros::Rate loop_rate(10);

  //Create a subscribers
  joint_states_sub = node.subscribe("/robot/joint_states", 100, &baxter_kdl::joint_state_callback, this);
  std::string arm_states_topic_name = "/robot/limb/" + limb +"/gravity_compensation_torques";
  arm_states_sub = node.subscribe(arm_states_topic_name.c_str(), 100, &baxter_kdl::arm_state_callback, this);

  //Read the URDF from param sever  
  if (!kdl_parser::treeFromParam("robot_description", kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }  

  //Get chain from kdl tree
  std::string base_link = "base";
  std::string tip_link = limb + "_gripper";  		// _gripper frame
  kdl_tree.getChain(base_link, tip_link, arm_chain);
  //Get number of Joints
  num_jnts = arm_chain.getNrOfJoints();

  // Some initializations
  new_time = ros::Time::now();
  q_current.resize(num_jnts);
  q_dot_current.resize(num_jnts);
  q_ddot_current.resize(num_jnts);
  q_effort_current.resize(num_jnts);
  external_torque.resize(num_jnts);
  default_threshold.resize(num_jnts);
  threshold.resize(num_jnts);

  for(int i=0;i<num_jnts;i++)
  {
    default_threshold[i] = default_threshold_[i];
  }
  threshold = default_threshold;

  last_q_current.resize(num_jnts);
  last_q_dot_current.resize(num_jnts);
  
  is_joint_state_ready = 0;
  // Wait for first message from joint_state subscriber arrives
  while(ros::ok() && !is_joint_state_ready)
  {
    ros::spinOnce();
  }
}

void baxter_kdl::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //setting joints in the right order to feed the jacobian
	//JointState order -> e0, e1, s0, s1, w0, w1, w2
	//desired order -> s0, s1, e0, e1, w0, w1, w2

  if(msg->name.size() == 1)
  {
    // filter out gripper joint_states
    return;
  }

  old_time = new_time;
  new_time = msg->header.stamp;
  time_step = new_time-old_time;
  for(int i=0; i<num_jnts;i++)
  {
    last_q_current[i] = q_current[i];
    last_q_dot_current[i] = q_dot_current[i];
  }

  int t = 0;
  if (limb=="right")
  {
    t = 7;
  }
	for(int i = 0; i < 2 ; i++)
  {
		// s0 and s1
    q_current[i] = (double)(msg->position[i + 4 + t]);
    q_dot_current[i] = (double)(msg->velocity[i + 4 + t]);
    q_effort_current[i] = (double)(msg->effort[i + 4 + t]);
	}

	for(int i = 2; i < 4 ; i++)
  { 
		// e0 and e1
    q_current[i] = (double)(msg->position[i + t]);
    q_dot_current[i] = (double)(msg->velocity[i + t]);
    q_effort_current[i] = (double)(msg->effort[i + t]);
	}

	for(int i = 4; i < 7 ; i++)
  { 
		// w0, w1, w2
    q_current[i] = (double)(msg->position[i + 2 + t]);
    q_dot_current[i] = (double)(msg->velocity[i + 2 + t]);
    q_effort_current[i] = (double)(msg->effort[i + 2 + t]);
	}

  // TODO: compute joint accelerations with Kalman Filter.
  for(int i=0; i<num_jnts;i++)
  {
    // dirty accelerations
    q_ddot_current[i] = (q_dot_current[i] - last_q_dot_current[i]) / time_step.toSec();
  }

  is_joint_state_ready = 1;
  compute_external_torque();  // update external torque
  collision_state();          // update collision state
}

void baxter_kdl::arm_state_callback(const baxter_core_msgs::SEAJointState::ConstPtr& msg)
{
  // this arm_state is slower than joint_states, but it can supply the hysteresis_model_effort and crosstalk_model_effort: (spring effort for s1 joint)
  spring_effort = msg->hysteresis_model_effort[1] - msg->crosstalk_model_effort[1];
}

bool baxter_kdl::forward_kinematics(vector<double> &result)
{
  return forward_kinematics(result, q_current);
}

bool baxter_kdl::forward_kinematics(vector<double> &result, vector<double> joint_values)
{
  result.resize(7); //  x y z ox oy oz ow
  if(joint_values.size()!=num_jnts)
  {
    return false;
  }
  //Create KDL Solver
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(arm_chain);
  //Create Joint Array for calculation
  KDL::JntArray	joint_positions = JntArray(num_jnts);

  for(int i = 0; i<num_jnts; i++)
  {
    joint_positions(i)=joint_values[i];
  }

  //Create the frame that will contain the results
  KDL::Frame end_frame;
  bool fk_status;
  fk_status = fksolver.JntToCart(joint_positions,end_frame);
  KDL::Vector pos;
  KDL::Rotation rotation = Rotation(end_frame.M);
  pos = end_frame.p;
  double rot[3];
  rotation.GetQuaternion(rot[0],rot[1],rot[2],rot[3]);

  //Modifying the output array
  for(int i=0; i<3; i++)
  {
    result[i]=pos(i);
  }
  for(int i=0; i<4; i++)
  {
    result[i+3]=rot[i];
  }
  return true;
}

bool baxter_kdl::inverse_kinematics(vector<double> &result, vector<double> position)
{
  result.resize(num_jnts);
  if(position.size() != 3)
  {
    return false;
  }
  vector<double> orientation;
  orientation.clear();    // zero size orientation
  return inverse_kinematics(result, position, orientation);
}

bool baxter_kdl::inverse_kinematics(vector<double> &result, vector<double> position, vector<double> orientation)
{
  result.resize(num_jnts);
  if(orientation.size() != 4 &&  orientation.size() !=0)
  {
    return false;
  }
  vector<double> seed;
  seed.resize(num_jnts);
  // Populate seed with current angles
  for(int i=0; i<num_jnts; i++)
  {
      seed[i]=q_current[i];
  }
  return inverse_kinematics(result, position, orientation, seed);
}

bool baxter_kdl::inverse_kinematics(vector<double> &result, vector<double> position, vector<double> orientation, vector<double> seed)
{
  result.resize(num_jnts);
  if(seed.size() != num_jnts)
  {
    return false;
  }
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(arm_chain);
  ChainIkSolverVel_pinv iksolver_v = 	ChainIkSolverVel_pinv(arm_chain);
  ChainIkSolverPos_NR iksolver_p = ChainIkSolverPos_NR(arm_chain,fksolver,iksolver_v);
  KDL::Vector pos = Vector(position[0],position[1],position[2]);
  KDL::Rotation rot = Rotation();
  if (orientation.size() != 0){
    rot = rot.Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]);
  }
  
  KDL::JntArray seed_array = JntArray(num_jnts);

  for(int i=0; i<num_jnts; i++)
  {
    seed_array(i)=seed[i];
  }
  
  //Make IK Call
  KDL::Frame goal_pose;
  if (orientation.size() != 0)
  {
    goal_pose = Frame(rot,pos);
  }
  else{
    goal_pose = Frame(pos);
  }
  KDL::JntArray result_angles = JntArray(num_jnts);
  bool ik_status;
  ik_status = iksolver_p.CartToJnt(seed_array, goal_pose, result_angles);  

  for(int i=0; i<7; i++)
  {
    result[i]=result_angles(i);
  }
  return true;
}

bool baxter_kdl::gravity_effort(vector<double> &result)
{
  // current gravity effort
  return gravity_effort(result, q_current);
}

bool baxter_kdl::gravity_effort(vector<double> &result, vector<double> joint_values)
{
  result.resize(num_jnts);
  if(joint_values.size() != num_jnts)
  {
    return false;
  }

  KDL::ChainDynParam dyn = KDL::ChainDynParam(arm_chain, KDL::Vector(0,0,-9.81));
  //Create Joint Array for calculation
  KDL::JntArray	joint_positions = JntArray(num_jnts);
  int i;

  for(i = 0; i<num_jnts; i++)
  {
    joint_positions(i)=joint_values[i];
  }
 
  KDL::JntArray	gravity = JntArray(num_jnts);		  // gravity array
  dyn.JntToGravity(joint_positions,gravity);

  // Modifying the output array 
  for(i=0; i<7; i++)
  {
    result[i]=gravity(i);
  }
  result[1] -= spring_effort;   // subtract spring effort from s1 joint
  return true;
}

bool baxter_kdl::inverse_dynamics(vector<double> &result)
{
  //TODO: calculate q_ddot_current
  for(int i=0;i<num_jnts;i++)
  {
    q_ddot_current[i] = 0;  // difference result is too bad.
  }

  return inverse_dynamics(result, q_current, q_dot_current, q_ddot_current);
}

bool baxter_kdl::inverse_dynamics(vector<double> &result, vector<double> joint_values, vector<double> joint_velocities, vector<double> joint_accelerations)
{
  result.resize(num_jnts);
  if(joint_values.size() != num_jnts)
  {
    return false;
  }

  // inverse dynamics solver: recursive newton euler solver
  KDL::ChainIdSolver_RNE idsolver = KDL::ChainIdSolver_RNE(arm_chain,KDL::Vector(0,0,-9.81));

  KDL::JntArray q = JntArray(num_jnts);
  KDL::JntArray q_dot = JntArray(num_jnts);
  KDL::JntArray q_dotdot = JntArray(num_jnts);
  std::vector<Wrench> wrenchnull;
  KDL::JntArray torques = JntArray(num_jnts);
  
  unsigned int ns = arm_chain.getNrOfSegments();
  for(int i=0;i<ns;i++)
  {
    wrenchnull.push_back(KDL::Wrench());
  }

  for(int i=0; i<num_jnts; i++)
  {
    q(i)=joint_values[i];
    q_dot(i)=joint_velocities[i];
    q_dotdot(i)=joint_accelerations[i];
  }
  
  int a = idsolver.CartToJnt(q, q_dot, q_dotdot, wrenchnull, torques);
  //Modifying the output array 
  for(int i=0; i<7; i++)
  {
    result[i]=torques(i);
  }
  result[1] -= spring_effort;   // subtract spring effort from e1 joint
  return true;
}

void baxter_kdl::compute_external_torque()
{
  vector<double> id_result;     // computed torque from inverse dynamics
  id_result.clear();
  
  inverse_dynamics(id_result);
  for(int i=0;i<num_jnts;i++)
  {
    external_torque[i] = q_effort_current[i] - id_result[i];
  }
}

void baxter_kdl::set_collision_threshold(vector<double> threshold_)
{
  threshold = threshold_;
}

void baxter_kdl::set_default_collision_threshold()
{
  threshold = default_threshold;
}

bool baxter_kdl::collision_state()
{
  is_collision = false;
  for(int i=0;i<num_jnts;i++)
  {
    if(fabs(external_torque[i])>threshold[i])
    {
      is_collision = true;
    }
  }
  return is_collision;
}

}

