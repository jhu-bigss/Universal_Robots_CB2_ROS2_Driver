#ifndef _DHKINEMATICS_H_
#define _DHKINEMATICS_H_

#include <cisstVector/vctFixedSizeVector.h>
#include <cisstVector/vctFixedSizeMatrix.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstVector/vctFixedSizeMatrixTypes.h>
#include <cisstVector/vctFrame4x4.h>
#include <cisstVector/vctTransformationTypes.h>

typedef vctFixedSizeMatrix<double, 4, 4> vctMatrix4x4;

inline vctFrm4x4 RotX(const double theta)
{
  vctFrm4x4 r;
  r.Rotation().From(vctAxAnRot3(vct3(1.0, 0.0, 0.0), theta));

  return r;
}

inline vctFrm4x4 RotY(const double theta)
{
  vctFrm4x4 r;
  r.Rotation().From(vctAxAnRot3(vct3(0.0, 1.0, 0.0), theta));

  return r;
}

inline vctFrm4x4 RotZ(const double theta)
{
  vctFrm4x4 r;
  r.Rotation().From(vctAxAnRot3(vct3(0.0, 0.0, 1.0), theta));

  return r;
}

inline vctFrm4x4 TransXYZ(const double x, const double y, const double z)
{
  vctFrm4x4 r;
  r.Translation().Assign(x, y, z);

  return r;
}


/// \brief
template <unsigned _NumJoints>
class DHKinematics
{
public:
  enum { kNUM_JOINTS = _NumJoints };

  typedef unsigned long size_type;

  typedef vctFixedSizeVector<double, kNUM_JOINTS> JointVector;
  typedef vctFixedSizeMatrix<double, 6, kNUM_JOINTS> JacobianMatrix;
  typedef vctFixedSizeVector<double, kNUM_JOINTS> JointVelocityVector;

  // TODO: Implement a ctor/setter methods for setting DH params, for now
  //       just do via a subclass

  /// \brief Computes the frame transformation from one coordinate frame
  ///        to the previous frame
  /// \param i The current frame
  /// \param theta The joint angle at frame frame i
  /// \return Homogeneous transformation matrix from i to i-1
  vctFrm4x4 dhf(const size_type i, const double theta) const;

  JacobianMatrix geo_jac(const JointVector& q, const vct3 offset = vct3(0.0)) const;

 // vctDoubleMat snake_jac(const JointVector& q, const vct3 offset = vct3(0.0), const double length_l = 0.0, const double length_r = 0.0) const;

  /// \brief Returns the pose of a frame with respect to the base (frame 0)
  /// \param q The current joint angles
  /// \param i The frame to compute the pose (defaults to the end-effector)
  /// \return Homogeneous transformation matrix representing the transformation
  ///         from frame i to the base
  vctMatrix4x4 pose(const JointVector& q, const size_type i = kNUM_JOINTS) const;

protected:
  // Standard DH parameters
  vctFixedSizeVector<double, kNUM_JOINTS> alpha_;
  vctFixedSizeVector<double, kNUM_JOINTS> a_;
  vctFixedSizeVector<double, kNUM_JOINTS> d_;
  vctFixedSizeVector<double, kNUM_JOINTS> theta_;

};

#include "DHKinematics-inl.h"

#endif // _DHKINEMATICS_H
