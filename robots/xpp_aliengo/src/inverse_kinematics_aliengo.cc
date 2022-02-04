#include <xpp_aliengo/inverse_kinematics_aliengo.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

namespace xpp {

Joints
InverseKinematicsAliengo::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Vector3d ee_pos_H;
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = x_B.ToImpl();
  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {

    AliengolegInverseKinematics::KneeBend bend = AliengolegInverseKinematics::Forward;

    using namespace quad;
    switch (ee) {
      case LF:
        ee_pos_H = pos_B.at(ee);
        break;
      case RF:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
        break;
      case LH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
        bend = AliengolegInverseKinematics::Backward;
        break;
      case RH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
        bend = AliengolegInverseKinematics::Backward;
        break;
      default: // joint angles for this foot do not exist
        break;
    }

    ee_pos_H -= base2hip_LF_;
    q_vec.push_back(leg.GetJointAngles(ee_pos_H, bend));
  }

  return Joints(q_vec);
}
}