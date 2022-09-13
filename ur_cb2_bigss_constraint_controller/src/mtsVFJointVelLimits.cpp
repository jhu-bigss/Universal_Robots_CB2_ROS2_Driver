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

#include <ur_cb2_bigss_constraint_controller/mtsVFJointVelLimits.h>

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFJointVelLimits::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double TickTime)
{    
    /*
         Fill in refs
         I*dq >= L 
         -I*dq >= -U 
    */

    mtsVFDataJointLimits * limitData = (mtsVFDataJointLimits*)(Data);

    size_t numLimits = limitData->LowerLimits.size();
    IneqConstraintMatrixRef.SetAll(0.0);

    for(size_t i = 0; i < numLimits; i++) {
        IneqConstraintMatrixRef.at(i,i) = 1.0;
        IneqConstraintMatrixRef.at(i+numLimits,i) = -1.0;
        IneqConstraintVectorRef.at(i) = limitData->LowerLimits.at(i);
        IneqConstraintVectorRef.at(i+numLimits) = -limitData->UpperLimits.at(i);
    }
}
