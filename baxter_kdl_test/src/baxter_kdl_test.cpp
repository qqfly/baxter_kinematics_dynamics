#include "baxter_kdl/baxter_kdl.h"

using namespace B_KDL;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_kdl_example");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  B_KDL::baxter_kdl kin = baxter_kdl("right");    // right arm
  
  KDL::Chain chain = kin.arm_chain;

  //std::cout<<"num of joints is "<<chain.getNrOfJoints()<<std::endl;
  //std::cout<<"num of links is "<<kin.arm_chain.getNrOfSegments()<<std::endl;
  int num_jnts = chain.getNrOfJoints();   // number of joints

  vector<double> fk_result;
  vector<double> ik_result;
  vector<double> grav_result;
  vector<double> id_result;

  //===================FK=======================
  printf("\nBaxter Position FK:\n");
  kin.forward_kinematics(fk_result);
  for(int i=0;i<num_jnts;i++)
  {
    printf("%f,",fk_result[i]);
  }
  printf("\n");

  //================IK target====================
  vector<double> position(3);
  vector<double> orientation(4);
  for(int i=0;i<3;i++)
  {
    position[i] = fk_result[i];
  }
  for(int i=0;i<4;i++)
  {
    orientation[i] = fk_result[3+i];
  }

  //=================IK with position=============
  printf("\nBaxter Position IK:\n");
  kin.inverse_kinematics(ik_result,position);
  for(int i=0;i<num_jnts;i++)
  {
    printf("%f,",ik_result[i]);
  }
  printf("\n");

  //===============IK with 6d pose=============
  printf("\nBaxter Pose IK:\n");
  kin.inverse_kinematics(ik_result,position,orientation);
  for(int i=0;i<num_jnts;i++)
  {
    printf("%f,",ik_result[i]);
  }
  printf("\n");

  //============IK  Validation====================
  printf("\nBaxter IK result validation:\n");
  kin.forward_kinematics(fk_result,ik_result);
  for(int i=0;i<num_jnts;i++)
  {
    printf("%f,",fk_result[i]);
  }
  printf("\n");

  //==============Gravity Torque=================
  printf("\nBaxter Gravity Effort:\n");
  kin.gravity_effort(grav_result);
  for(int i=0;i<num_jnts;i++)
  {
    printf("%f,",grav_result[i]);
  }
  printf("\n");

  //==========Inverse Dynamics Torque=============
  printf("\nBaxter ID Effort:\n");
  kin.inverse_dynamics(id_result);
  for(int i=0;i<num_jnts;i++)
  {
    printf("%f,",id_result[i]);
  }
  printf("\n");

  //==============Collision Check=================
  vector<double> collision_threshold;
  collision_threshold.resize(num_jnts);
  for(int i=0; i<num_jnts; i++)
  {
    collision_threshold[i] = 9;
  }
  int collision_times = 0;

  while(ros::ok())
  {
    printf("********************\n");
    //kin.set_collision_threshold(collision_threshold);  
    kin.set_default_collision_threshold();

    bool is_collision = kin.is_collision;

    printf("External torque is [");
    for(int i=0;i<7;i++)
    {
      //cout<<" "<<kin.external_torque[i];
      printf(" %5.2f",kin.external_torque[i]);
    }
    printf(" ]\n");
    
    if(is_collision)
    {
      collision_times ++;
      ROS_WARN("Baxter is in collision!");
      sleep(1);
      //return 0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  cout<<collision_times<<endl;
  return 0;
}
