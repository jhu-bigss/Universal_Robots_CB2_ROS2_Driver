/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 Author(s):  Paul Wilkening, Henry Phaeln
 Created on:

 (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFFixOrientation_h
#define _mtsVFFixOrientation_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/prmKinematicsState.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFFixOrientation: A class that contains logic for the implementation of virtual fixtures
 */
class mtsVFFixOrientation : public mtsVFBase
{
public:
    /*! Constructor
     */
    mtsVFFixOrientation() : mtsVFBase() {}
    /*! Constructor
    \param name String name of object
    */
    mtsVFFixOrientation(const std::string &name, mtsVFDataBase *data) : mtsVFBase(name, data) {}
    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
     */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);
    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime) {}
};

struct mtsVFDataFixOrientation : mtsVFDataBase
{
    prmKinematicsState *kinematics_for_last_joint_that_moves_frame_of_interest;
    vctFrm3 offset_from_kinematics_to_frame_of_interest;
    vctRot3 desired_orientation;
    int num_joints_system;

    mtsVFDataFixOrientation() : mtsVFDataBase() {}

    mtsVFDataFixOrientation(const mtsVFDataFixOrientation &other) : mtsVFDataBase(other) {}

    ~mtsVFDataFixOrientation() {}

    std::string HumanReadable(void) const
    {
        return std::string();
    }
};

#endif
