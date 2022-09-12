/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 Author(s):  Paul Wilkening
 Created on: 2015

 (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFJointVelLimits_h
#define _mtsVFJointVelLimits_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFBase.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFAbsoluteJointLimits.h>

/*! \brief mtsVFAbsoluteJointLimits: A class that contains logic for the implementation of joint velocity limits
 */
class mtsVFJointVelLimits : public mtsVFBase
{

public:
    /*! Constructor
     */
    mtsVFJointVelLimits() : mtsVFBase() {}

    /*! Constructor
    \param name String name of object
    */
    mtsVFJointVelLimits(const std::string &name,
                        mtsVFDataJointLimits *data) : mtsVFBase(name, data) {}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
     */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    // TODO: below are placeholders
    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime) {}
    void AssignRefs(const mtsVFBase::CONTROLLERMODE, const double, const vctDoubleVec &, vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &) {}
};

#endif // _mtsVFJointVelLimits_h
