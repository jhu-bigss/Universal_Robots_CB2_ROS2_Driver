#ifndef _URDHKINEMATICS_H_
#define _URDHKINEMATICS_H_

//BTX

#include "DHKinematics.h"
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctDynamicVectorTypes.h>

class URDHKinematics : public DHKinematics<6>
{
public:
  inline URDHKinematics()
  {
    set_ur5();
  }

  /// Set the UR5 DH kinematics
  void set_ur5();

  /// Set the UR10 DH kinematics
  void set_ur10();

  /// Compute the inverse kinematics given a pose.
  /// \param pose The end-effector pose
  /// \param q A solution vector, if one exists.
  /// \return The number of solutions
  int inverse_kinematics(const vctFrm4x4 &pose, vctDoubleMat &q);

  /// Choose the best joint positioins (from IK) such that they are
  /// closest to the current joint position
  /// \param ik The inverse kinematic solutions, on [0, 2*pi)
  /// \param currentJoints The current joint position
  /// \param closestJoints Populated with the best joint solution
  void find_closest_ik(const vctDoubleMat &ik,
    const vctDoubleVec &currentJoints, vctDoubleVec &joints);

  /// Choose the best joint positioins (from IK) such that they
  /// are closest to the current joint position
  /// \param ik The inverse kinematic solutions, on [0, 2*pi)
  /// \param currentJoints The current joint position
  /// \param closestJoints Populated with the best joint solution
  void find_closest_ik(const vctDoubleMat &ik,
    const vctFixedSizeVector<double, 6> &currentJoints,
    vctFixedSizeVector<double, 6> &joints);
};

// ETX

#endif
