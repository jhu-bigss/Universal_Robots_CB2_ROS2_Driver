#include "ur_cb2_robot_driver/ur/URDHKinematics.h"
#include "ur_cb2_robot_driver/ur/URDHParams.h"
#include <cmath>

const double PI = 3.14159265359;
const double tol = 0.000001;

/// Wrap an angle to [0, 2*pi)
double wrapAngle(const double x)
{
  return (x >= 0.0) ? x : ((x >= -tol) ? 0.0 : (x + (2 * PI)));
}

void URDHKinematics::set_ur5()
{
  URDHParams::set_ur5<kNUM_JOINTS>(alpha_, a_, d_, theta_);
}

void URDHKinematics::set_ur10()
{
  URDHParams::set_ur10<kNUM_JOINTS>(alpha_, a_, d_, theta_);
}

int URDHKinematics::inverse_kinematics(const vctFrm4x4 &pose, vctDoubleMat &q)
{
  int numSols = 0;
  q.SetSize(6, 8); // maximum number of solutions on [0, 2*pi]
  q.SetAll(0.0);

  ////////
  // q0 //
  ////////
  vct3 d6(0.0, 0.0, -d_[5]);
  vct3 p0_5 = pose * d6;
  double R = std::sqrt(p0_5.X() * p0_5.X() + p0_5.Y() * p0_5.Y());
  double d4_R = d_[3]/R;

  // there are no solutions here
  if(std::fabs(d4_R) > 1)
    return 0;

  double q0[2];
  double psi = std::atan2(p0_5.Y(), p0_5.X());
  double phi = std::acos(d4_R);
  q0[0] = wrapAngle(psi + phi + PI/2);
  q0[1] = wrapAngle(psi - phi + PI/2);

  ////////
  // q4 //
  ////////
  double q4[2][2];
  for (int i=0; i<2; i++) {
    double val = (pose.Translation().X() * std::sin(q0[i]) - pose.Translation().Y() * std::cos(q0[i]) - d_[3])/d_[5];
    if(std::fabs(val) > 1)
      return 0;

    q4[i][0] = wrapAngle(std::acos(val));
    q4[i][1] = wrapAngle(-q4[i][0]);
  }

  //////////
  // Rest //
  //////////
  for (int i=0; i<2; i++) {
    double q5;
    vctFrm4x4 T0_1 = dhf(0, q0[i]);
    vctFrm4x4 T1_6 = T0_1.Inverse() * pose;
    vctFrm4x4 T6_1 = pose.Inverse() * T0_1;

    for (int j=0; j<2; j++) {
      double s4 = std::sin(q4[i][j]);

      // If joints 2, 3, 4, and 6 are parallel
      if(std::fabs(s4) < tol)
        q5 = 0.0; // can match any rotation, set desired to 0.0
      else
        q5 = wrapAngle(std::atan2(-T6_1.Rotation().Element(1, 2) * s4, T6_1.Rotation().Element(0, 2) * s4));

      vctFrm4x4 T4_6 = dhf(4, q4[i][j]) * dhf(5, q5);
      vctFrm4x4 T1_4 = T1_6 * T4_6.Inverse();
      vct3 p1_3 = T1_4*vct3(0.0, -d_[3], 0.0);
      double normP = p1_3.Norm();
      double c3 = (normP*normP - a_[1]*a_[1]-a_[2]*a_[2])/(2*a_[1]*a_[2]);
      if(std::fabs(c3) > 1.0)
        continue;

      double q2[2];
      q2[0] = wrapAngle(std::acos(c3));
      q2[1] = wrapAngle(-q2[0]);

      for (int k=0; k<2; k++) {
        double asin_eps = a_[2]*sin(q2[k])/normP;
        if(std::fabs(asin_eps) > 1)
          continue;

        double q1 = wrapAngle(-std::atan2(p1_3.Y(), -p1_3.X()) + std::asin(asin_eps));

        vctFrm4x4 T1_3 = dhf(1, q1) * dhf(2, q2[k]);
        vctFrm4x4 T3_4 = T1_3.Inverse() * T1_4;
        double q3 = wrapAngle(std::atan2(T3_4.Rotation().Element(1, 0), T3_4.Rotation().Element(0, 0)));

        q(0, numSols) = q0[i];
        q(1, numSols) = q1;
        q(2, numSols) = q2[k];
        q(3, numSols) = q3;
        q(4, numSols) = q4[i][j];
        q(5, numSols) = q5;
        numSols ++;
      }
    }
  }

  return numSols;
}

void URDHKinematics::find_closest_ik(const vctDoubleMat &ik, const vctDoubleVec &currentJoints, vctDoubleVec &joints)
{
  vctFixedSizeVector<double, 6> cjFixed, jFixed;
  cjFixed.Assign(currentJoints);

  find_closest_ik(ik, cjFixed, jFixed);

  joints.SetSize(6);
  joints.Assign(jFixed);
}

void URDHKinematics::find_closest_ik(const vctDoubleMat &ik, const vctFixedSizeVector<double, 6> &currentJoints,
    vctFixedSizeVector<double, 6> &joints)
{
  vct::size_type nSols = ik.cols();
  joints.Assign(ik.Column(0));

  // create the distance map
  vctFixedSizeVector<double, 6> currGuess;

  // just brute force this
  // there are probably some clever routines, but the number of elements is small
  double d1, d2, d, bestD;
  bestD = 1000;
  for(vct::size_type j=0; j<nSols; j++) {
    d = 0;
    currGuess.Assign(ik.Column(j));
    for(unsigned int i=0; i<6; i++) {
      d1 = ik(i,j) - currentJoints(i);
      d2 = d1 - 2*PI;
      if(d1 < 0)
        d1 *= -1;
      if(d2 < 0)
        d2 *= -1;

  
      if(d1 < d2)
        d += d1;
      else {
        d += d2;
        currGuess(i) = ik(i,j) - 2*PI;
      }
    }
    if(d < bestD) {
      bestD = d;
      joints.Assign(currGuess);
    }
  }
}

