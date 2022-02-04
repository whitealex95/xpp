#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_aliengo/inverse_kinematics_aliengo.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "aliengo_urdf_visualizer");

  const std::string joint_desired_aliengo = "xpp/joint_aliengo_des";

  auto aliengo_ik = std::make_shared<InverseKinematicsAliengo>();
  CartesianJointConverter inv_kin_converter(aliengo_ik,
                  xpp_msgs::robot_state_desired,
                  joint_desired_aliengo);
  
  // urdf joint names
  int n_ee = aliengo_ik->GetEECount();
  int n_j  = AliengolegJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + HAA) = "FL_hip_joint";
  joint_names.at(n_j*LF + HFE) = "FL_thigh_joint";
  joint_names.at(n_j*LF + KFE) = "FL_calf_joint";
  joint_names.at(n_j*RF + HAA) = "FR_hip_joint";
  joint_names.at(n_j*RF + HFE) = "FR_thigh_joint";
  joint_names.at(n_j*RF + KFE) = "FR_calf_joint";
  joint_names.at(n_j*LH + HAA) = "RL_hip_joint";
  joint_names.at(n_j*LH + HFE) = "RL_thigh_joint";
  joint_names.at(n_j*LH + KFE) = "RL_calf_joint";
  joint_names.at(n_j*RH + HAA) = "RR_hip_joint";
  joint_names.at(n_j*RH + HFE) = "RR_thigh_joint";
  joint_names.at(n_j*RH + KFE) = "RR_calf_joint";

  std::string urdf = "aliengo_rviz_urdf_robot_description";
  UrdfVisualizer aliengo_desired(urdf, joint_names, "base", "world",
              joint_desired_aliengo, "aliengo_des");
  
  ::ros::spin();

  return 1;
}