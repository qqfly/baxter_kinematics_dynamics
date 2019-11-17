#include <baxter_kdl/baxter_kdl.h>

using namespace B_KDL;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_kdl_example");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  B_KDL::BaxterDynamics kin("right"); //// right arm

  KDL::Chain chain = kin.m_arm_chain;

  int num_jnts = chain.getNrOfJoints(); // number of joints

  std::vector<double> fk_result;
  std::vector<double> ik_result;
  std::vector<double> grav_result;
  std::vector<double> id_result;

  //===================FK=======================
  printf("\nBaxter Position FK:\n");
  kin.ForwardKinematics(fk_result);
  for (size_t i = 0; i < num_jnts; i++)
  {
    printf("%f,", fk_result[i]);
  }
  printf("\n");

  //================IK target====================
  std::vector<double> position(3);
  std::vector<double> orientation(4);
  for (int i = 0; i < 3; i++)
  {
    position[i] = fk_result[i];
  }
  for (int i = 0; i < 4; i++)
  {
    orientation[i] = fk_result[3 + i];
  }

  //=================IK with position=============
  printf("\nBaxter Position IK:\n");
  kin.InverseKinematics(position, ik_result);
  for (size_t i = 0; i < num_jnts; i++)
  {
    printf("%f,", ik_result[i]);
  }
  printf("\n");

  //===============IK with 6d pose=============
  printf("\nBaxter Pose IK:\n");
  kin.InverseKinematics(position, orientation, ik_result);
  for (size_t i = 0; i < num_jnts; i++)
  {
    printf("%f,", ik_result[i]);
  }
  printf("\n");

  //============IK  Validation====================
  printf("\nBaxter IK result validation:\n");
  kin.ForwardKinematics(fk_result, ik_result);
  for (size_t i = 0; i < num_jnts; i++)
  {
    printf("%f,", fk_result[i]);
  }
  printf("\n");

  //==============Gravity Torque=================
  printf("\nBaxter Gravity Effort:\n");
  kin.GravityEffort(grav_result);
  for (size_t i = 0; i < num_jnts; i++)
  {
    printf("%f,", grav_result[i]);
  }
  printf("\n");

  //==========Inverse Dynamics Torque=============
  printf("\nBaxter ID Effort:\n");
  kin.InverseDynamics(id_result);
  for (size_t i = 0; i < num_jnts; i++)
  {
    printf("%f,", id_result[i]);
  }
  printf("\n");

  //==============Collision Check=================
  std::vector<double> collision_threshold;
  collision_threshold.resize(num_jnts);
  for (size_t i = 0; i < num_jnts; i++)
  {
    collision_threshold[i] = 9;
  }
  int collision_times = 0;

  while (ros::ok())
  {
    printf("********************\n");
    //kin.SetCollisionThreshold(collision_threshold);
    kin.SetDefaultCollisionThreshold();

    printf("External torque is [");
    for (size_t i = 0; i < num_jnts; i++)
    {
      //cout<<" "<<kin.m_external_torque[i];
      printf(" %5.2f", kin.m_external_torque[i]);
    }
    printf(" ]\n");

    if (kin.m_is_collision)
    {
      collision_times++;
      ROS_WARN("Baxter is in collision!");
      sleep(1);
      //return 0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  cout << collision_times << endl;
  return 0;
}
