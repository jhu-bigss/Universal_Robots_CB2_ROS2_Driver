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

#include <ur_cb2_bigss_constraint_controller/mtsVFAbsoluteJointLimits.h>

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFAbsoluteJointLimits::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double TickTime)
{    
    /*
         Fill in refs
         I*dq >= L - q
         -I*dq >= -U + q
    */
    prmKinematicsState* CurrentKinematics = Kinematics.at(0);

    mtsVFDataAbsoluteJointLimits * limitData = (mtsVFDataAbsoluteJointLimits*)(Data);

    size_t numJoints = limitData->LowerLimits.size();
    IneqConstraintMatrixRef.SetAll(0.0);

    for(size_t i = 0; i < numJoints; i++)
    {        
        IneqConstraintMatrixRef.at(i,i) = 1.0;
        IneqConstraintMatrixRef.at(i+numJoints,i) = -1.0;
        IneqConstraintVectorRef.at(i) = limitData->LowerLimits.at(i)
          - CurrentKinematics->JointState->Position()[i];
        IneqConstraintVectorRef.at(i+numJoints) = -limitData->UpperLimits.at(i)
          + CurrentKinematics->JointState->Position()[i];
    }
}

// void mtsVFAbsoluteJointLimits::ConvertRefs(){}
