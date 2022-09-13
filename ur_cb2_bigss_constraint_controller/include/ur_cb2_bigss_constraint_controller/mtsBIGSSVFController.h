/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening, Henry Phalen

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

This is just an extension on the sawConstraintController mtsVFController.
I would like to see whatever useful functionality just be ported back into
sawConstraintController
*/

#ifndef _mtsBIGSSVFController_h
#define _mtsBIGSSVFController_h

#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFBase.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFollowSettlingGains.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFAbsoluteJointLimits.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFStayOnAxis.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFixOrientation.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFJointPenalty.h>
#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFRCM.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFollowPositionTrajectory.h>

class mtsBIGSSVFController : public mtsVFController
{

public:
    mtsBIGSSVFController(size_t numJoints = 0, mtsVFBase::CONTROLLERMODE mode = mtsVFBase::JPOS);

    ~mtsBIGSSVFController() {}

    // Debugging functions
    void PrintProblem();
    void PrintSensors();
    void PrintKinematics();
    void PrintConstraints();

    void AddVFStayOnAxis(mtsVFDataStayOnAxis &vf);
    void AddVFFollowSettlingGains(mtsVFDataFollowGains &vf);
    void AddVFJointVelLimits(mtsVFDataJointLimits &vf);
    void AddVFAbsoluteJointLimits(mtsVFDataAbsoluteJointLimits &vf);
    void AddVFRCM(mtsBIGSSVFDataRCM &vf);
    void AddVFJointPenalty(mtsVFDataJointPenalty &vf);
    void AddVFFixOrientation(mtsVFDataFixOrientation &vf);
    void AddVFFollowPositionTrajectory(mtsVFDataFollowPositionTrajectory &vf);

    bool Solve(vctDoubleVec &dq);

};

#endif // _mtsBIGSSVFController_h
