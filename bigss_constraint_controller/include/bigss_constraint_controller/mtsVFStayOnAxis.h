/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 Author(s):  Paul Wilkening, Henry Phalen
 Created on:

 (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFStayOnAxis_h
#define _mtsVFStayOnAxis_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/prmKinematicsState.h>

class mtsVFStayOnAxis : public mtsVFBase
{
public:
    mtsVFStayOnAxis() : mtsVFBase() {}

    mtsVFStayOnAxis(const std::string &name, mtsVFDataBase *data) : mtsVFBase(name, data) {}

    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);
    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime) {}
};

struct mtsVFDataStayOnAxis : mtsVFDataBase
{
    prmKinematicsState *kinematics_for_last_joint_that_moves_frame_of_interest;
    vctFrm3 offset_from_kinematics_to_frame_of_interest;
    vct3 desired_axis;
    vct3 point_on_desired_axis;
    int num_joints_system;

    mtsVFDataStayOnAxis() : mtsVFDataBase() {}

    mtsVFDataStayOnAxis(const mtsVFDataStayOnAxis &other) : mtsVFDataBase(other) {}

    ~mtsVFDataStayOnAxis() {}

    std::string HumanReadable(void) const
    {
        return std::string();
    }
};

#endif
