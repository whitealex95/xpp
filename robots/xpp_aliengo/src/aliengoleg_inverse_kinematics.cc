#include <xpp_aliengo/aliengoleg_inverse_kinematics.h>

#include <cmath>
#include <map>
#include <iostream>

#include <xpp_states/cartesian_declarations.h>

namespace xpp {


AliengolegInverseKinematics::Vector3d
AliengolegInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B) const
{
  double q_HAA_bf, q_HAA_br, q_HFE_br; // rear bend of knees
  double q_HFE_bf, q_KFE_br, q_KFE_bf; // forward bend of knees

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  xr = ee_pos_B;
  double tmp1 = pow(xr[Y], 2) + pow(xr[Z], 2);
  double alpha1 = acos(0.083/sqrt(tmp1));
  double alpha2 = atan2(-xr[Z], xr[Y]);
  // compute the HAA angle
  q_HAA_bf = q_HAA_br = alpha1 - alpha2;

  // rotate into the HFE coordinate system (rot around X)
  R <<  1.0, 0.0, 0.0, 
        0.0, cos(-q_HAA_bf), -sin(-q_HAA_bf),
        0.0, sin(-q_HAA_bf), cos(-q_HAA_bf);
  
  xr =  (R * xr).eval();

  // compute square of length from HFE to foot
  double tmp2 = pow(xr[X], 2)+pow(xr[Z],2);

  // compute temporary angles (with reachability check)
  double lu = length_thigh; // length of upper leg
  double ll = length_shank; // length of lower leg
  
  double beta1 = atan2(-xr[Z],xr[X]) - 0.5*M_PI; // flip and rotate to match HyQ joint definition

  double some_random_value_for_beta = (pow(lu,2)+tmp2-pow(ll,2))/(2.*lu*sqrt(tmp2));
  if (some_random_value_for_beta > 1) {
    some_random_value_for_beta = 1;
  }
  if (some_random_value_for_beta < -1) {
    some_random_value_for_beta = -1;
  }
  double beta2 = acos(some_random_value_for_beta);

  // compute Hip FE angle
  q_HFE_bf = q_HFE_br = beta1 + beta2;

  double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp2)/(2.*ll*lu);
  // law of cosines give the knee angle
  if (some_random_value_for_gamma > 1) {
    some_random_value_for_gamma = 1;
  }
  if (some_random_value_for_gamma < -1) {
    some_random_value_for_gamma = -1;
  }
  double gamma  = acos(some_random_value_for_gamma);


  q_KFE_bf = q_KFE_br = gamma - M_PI;

  // // forward knee bend
  // EnforceLimits(q_HAA_bf, HAA);
  // EnforceLimits(q_HFE_bf, HFE);
  // EnforceLimits(q_KFE_bf, KFE);

  // // backward knee bend
  // EnforceLimits(q_HAA_br, HAA);
  // EnforceLimits(q_HFE_br, HFE);
  // EnforceLimits(q_KFE_br, KFE);

  return Vector3d(q_HAA_bf, q_HFE_bf, q_KFE_bf);

}

void
AliengolegInverseKinematics::EnforceLimits (double& val, AliengoJointID joint) const
{
  // totally exaggerated joint angle limits
  // totally exaggerated joint angle limits
  const static double haa_min = -180;
  const static double haa_max =  90;

  const static double hfe_min = -90;
  const static double hfe_max =  90;

  const static double kfe_min = -180;
  const static double kfe_max =  0;

  // reduced joint angles for optimization
  static const std::map<AliengoJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduceed joint angles for optimization
  static const std::map<AliengoJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}
} /* namespace xpp */