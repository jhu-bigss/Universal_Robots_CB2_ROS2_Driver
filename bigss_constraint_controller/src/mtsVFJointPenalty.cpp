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

#include <bigss_constraint_controller/mtsVFJointPenalty.h>

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFJointPenalty::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double TickTime)
{    
    /*
         We are adding a proportional penalty to each joint motion
    */
    mtsVFDataJointPenalty * JointPenaltyData = (mtsVFDataJointPenalty*)(Data);

    for (size_t i =0; i< Data->ObjectiveRows; i++){
      ObjectiveMatrixRef.at(i,i) = JointPenaltyData->JointPenalty[i];
    }
     
}
