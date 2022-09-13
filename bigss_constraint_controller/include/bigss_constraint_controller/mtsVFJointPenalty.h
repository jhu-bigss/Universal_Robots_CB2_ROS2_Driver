/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 Author(s):  Paul Wilkening, Henry Phalen
 Created on: 2015

 (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFJointPenalty_h
#define _mtsVFJointPenalty_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctTransformationTypes.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFBase.h>

struct mtsVFDataJointPenalty : mtsVFDataBase
{
    vctDoubleVec JointPenalty;

    mtsVFDataJointPenalty(void) : mtsVFDataBase() {}
    mtsVFDataJointPenalty(const mtsVFDataJointPenalty &other) : mtsVFDataBase(other), JointPenalty(other.JointPenalty) {}
    ~mtsVFDataJointPenalty() {}

    std::string HumanReadable(void) const
    {
        std::stringstream description__cdg;
        description__cdg << "mtsVFDataJointPenalty" << std::endl;
        description__cdg << cmnData<mtsVFDataBase>::HumanReadable(*this) << std::endl;
        description__cdg << "  JointPenalty:" << cmnData<vctDoubleVec>::HumanReadable(this->JointPenalty);
        return description__cdg.str();
    }
};

/*! \brief mtsVFJointPenalty: A class that contains logic for the implementation of joint limits
 */
class mtsVFJointPenalty : public mtsVFBase
{

public:
    /*! Constructor
     */
    mtsVFJointPenalty() : mtsVFBase() {}

    /*! Constructor
    \param name String name of object
    */
    mtsVFJointPenalty(const std::string &name,
                      mtsVFDataJointPenalty *data) : mtsVFBase(name, data) {}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
     */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime) {}
    void AssignRefs(const mtsVFBase::CONTROLLERMODE, const double, const vctDoubleVec &, vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &) {}
};

#endif // _mtsVFJointPenalty_h
