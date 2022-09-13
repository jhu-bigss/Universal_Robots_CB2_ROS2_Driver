/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id: $

 Author(s):  Paul Wilkening, Henry Phalen
 Created on:

 (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFFollowSettlingGains_h
#define _mtsVFFollowSettlingGains_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFFollow.h>
#include <cisstVector/vctTransformationTypes.h>
#include <sawConstraintController/mtsVFDataBase.h>

/*! @brief Calculate joint angles that take the robot from the current position and orientation to a desired one.

    This is useful for having the arm follow a path,
    particularly one that leads the end effector slightly
    at each instant to indicate the new destination.

    fill in refs
    min || J*dq - [v_T ; v_R] ||
    v_T = p_des - R_des * R^(-1)_cur * p_cur
    v_R = sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    J is the jacobian, p_des is the desired frame's translation,
    R_des is the desired frame's rotation, R^(-1)_cur is the inverse of the
    current frame's rotation,
    p_cur is the current frame's translation, A is an arbitrary vector,
    sk(A)^(-1) is the inverse of the skew matrix of A

   The goal is constructing the dx parameter which contains:
   [x, y, z, rx*theta, ry*theta, rz*theta]
   This is the same xdot format output by J*qdot
   that way we try to make xdot - J*qdot == [0,0,0,0,0,0].transpose()
   which means that you've found the joint angles that
   will put you at exactly the desired position and rotation
 */
struct mtsVFDataFollowGains : mtsVFDataBase
{
    vct3 gains;
    vctMat weights;

    mtsVFDataFollowGains(void);
    mtsVFDataFollowGains(const mtsVFDataFollowGains &other);
    ~mtsVFDataFollowGains();
    std::string HumanReadable(void) const;
};

class mtsVFFollowSettlingGains : public mtsVFBase
{

public:
    prmKinematicsState *CurrentKinematics;
    prmKinematicsState *DesiredKinematics;

    double gain_p, gain_d;
    vct3 prev_dx;
    vct3 prev_dx_rotation;
    vct3 int_dx;
    vct3 int_dx_rot;
    double angle_sum = 0.0;

    double angle_last = NAN;
    vct3 pos_sum{0.0, 0.0, 0.0};
    vct3 pos_last{NAN, NAN, NAN};

    vct3 rot_sum{0.0, 0.0, 0.0};
    vct3 rot_last{NAN, NAN, NAN};

    bool first_iter = true;
    /*! Constructor
     */
    mtsVFFollowSettlingGains() : mtsVFBase() {}

    /*! Constructor
    \param name String name of object
    */
    mtsVFFollowSettlingGains(const std::string &name, mtsVFDataFollowGains *data)
        : mtsVFBase(name, data) {}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
     */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);
};

#endif
