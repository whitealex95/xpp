#ifndef XPP_VIS_INVERSEKINEMATICS_ALIENGO_H_
#define XPP_VIS_INVERSEKINEMATICS_ALIENGO_H_

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_aliengo/aliengoleg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics functino for the Aliengo robot
 */
class InverseKinematicsAliengo : public InverseKinematics {
public:
  InverseKinematicsAliengo() = default;
  virtual ~InverseKinematicsAliengo() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position
   * @param pos_B 3D-position of the foot expressed in the base frame
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects
   */
  int GetEECount() const override { return 4; };

private:
  Vector3d base2hip_LF_ = Vector3d(0.3735, 0.207, 0.0);
  AliengolegInverseKinematics leg;
};

} /* namespace xpp */

#endif /* XPP_VIS_INVERSEKINEMATICS_ALIENGO_H_ */