#ifndef URDHPARAMS_H
#define URDHPARAMS_H

namespace URDHParams
{

  template <unsigned _NumJoints> void set_ur5(
      vctFixedSizeVector<double, _NumJoints> &alpha,
      vctFixedSizeVector<double, _NumJoints> &a,
      vctFixedSizeVector<double, _NumJoints> &d,
      vctFixedSizeVector<double, _NumJoints> &theta)
  {
     const double kPI = 3.141592653589793;

     // It appears that alpha, a, and d were all obtained by reading
     // the values directly from packets sent from the ur. I have since
     // deleted the print statements in the RobotServer class, but if
     // you checkout commit d5d21e48c4f9b you can see them in the Run
     // method.
      alpha[0] =  kPI / 2;
      alpha[1] =  0;
      alpha[2] =  0;
      alpha[3] =  kPI / 2;
      alpha[4] = -kPI / 2;
      alpha[5] =  0;

      a[0] =  0;
      a[1] = -0.425;
      a[2] = -0.39225;
      a[3] =  0;
      a[4] =  0;
      a[5] =  0;

      d[0] = 0.08959;
      d[1] = 0;
      d[2] = 0;
      d[3] = 0.10915;
      d[4] = 0.09465;
      d[5] = 0.0823;

      theta[0] = 0.0;
      theta[1] = 0.0;
      theta[2] = 0.0;
      theta[3] = 0.0;
      theta[4] = 0.0;
      theta[5] = 0.0;
  }

  template <unsigned _NumJoints> void set_ur10(
      vctFixedSizeVector<double, _NumJoints> &alpha,
      vctFixedSizeVector<double, _NumJoints> &a,
      vctFixedSizeVector<double, _NumJoints> &d,
      vctFixedSizeVector<double, _NumJoints> &theta)
  {
    const double kPI = 3.141592653589793;

    alpha[0] =  kPI / 2;
    alpha[1] =  0;
    alpha[2] =  0;
    alpha[3] =  kPI / 2;
    alpha[4] = -kPI / 2;
    alpha[5] =  0;

    a[0] =  0;
    a[1] = -0.612;
    a[2] = -0.5723;
    a[3] =  0;
    a[4] =  0;
    a[5] =  0;

    d[0] = 0.1273;
    d[1] = 0;
    d[2] = 0;
    d[3] = 0.163941;
    d[4] = 0.1157;
    d[5] = 0.0922;

    theta[0] = 0.0;
    theta[1] = 0.0;
    theta[2] = 0.0;
    theta[3] = 0.0;
    theta[4] = 0.0;
    theta[5] = 0.0;
  }

}

#endif

