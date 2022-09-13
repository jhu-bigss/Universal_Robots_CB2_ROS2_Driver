/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening, Rachel Hegeman, Henry Phalen

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFBase.h>
#include <bigss_constraint_controller/mtsVFAbsoluteJointLimits.h>
#include <bigss_constraint_controller/mtsBIGSSVFRCM.h>
#include <bigss_constraint_controller/mtsBIGSSVFController.h>
#include <bigss_constraint_controller/mtsVFJointPenalty.h>
#include <bigss_constraint_controller/mtsVFStayOnAxis.h>
#include <bigss_constraint_controller/mtsVFFixOrientation.h>
#include <bigss_constraint_controller/mtsVFFollowPositionTrajectory.h>
#include <bigss_constraint_controller/mtsVFJointVelLimits.h>


mtsBIGSSVFController::mtsBIGSSVFController(size_t numJoints, mtsVFBase::CONTROLLERMODE mode) :
  mtsVFController(numJoints, mode) {}

void mtsBIGSSVFController::AddVFFollowPositionTrajectory(mtsVFDataFollowPositionTrajectory &vf){
  if (!SetVFData(vf))
  {VFMap.insert(std::pair<std::string,mtsVFFollowPositionTrajectory *>(vf.Name,new mtsVFFollowPositionTrajectory(vf.Name, &vf)));}
}

void mtsBIGSSVFController::AddVFStayOnAxis(mtsVFDataStayOnAxis &vf){
  if (!SetVFData(vf))
    {VFMap.insert(std::pair<std::string,mtsVFStayOnAxis *>(vf.Name,new mtsVFStayOnAxis(vf.Name, &vf)));}
}

void mtsBIGSSVFController::AddVFFixOrientation(mtsVFDataFixOrientation &vf){
  if (!SetVFData(vf))
    {VFMap.insert(std::pair<std::string,mtsVFFixOrientation *>(vf.Name,new mtsVFFixOrientation(vf.Name, &vf)));}
}

void mtsBIGSSVFController::AddVFFollowSettlingGains(mtsVFDataFollowGains &vf){
  if (!SetVFData(vf))
  {VFMap.insert(std::pair<std::string,mtsVFFollowSettlingGains *>(vf.Name,new mtsVFFollowSettlingGains(vf.Name, &vf)));}
}

void mtsBIGSSVFController::AddVFJointVelLimits(mtsVFDataJointLimits & vf){
  if (!SetVFData(vf))
        VFMap.insert(std::pair<std::string,mtsVFJointVelLimits *>(vf.Name,new mtsVFJointVelLimits(vf.Name, &vf)));
}

void mtsBIGSSVFController::AddVFAbsoluteJointLimits(mtsVFDataAbsoluteJointLimits & vf){
  if (!SetVFData(vf))
        VFMap.insert(std::pair<std::string,mtsVFAbsoluteJointLimits *>(vf.Name,new mtsVFAbsoluteJointLimits(vf.Name, &vf)));
}

void mtsBIGSSVFController::AddVFRCM(mtsBIGSSVFDataRCM & vf){
  if (!SetVFData(vf))
        VFMap.insert(std::pair<std::string,mtsBIGSSVFRCM *>(vf.Name,new mtsBIGSSVFRCM(vf.Name, &vf)));
}

void mtsBIGSSVFController::AddVFJointPenalty(mtsVFDataJointPenalty & vf){
  if (!SetVFData(vf))
        VFMap.insert(std::pair<std::string,mtsVFJointPenalty *>(vf.Name,new mtsVFJointPenalty(vf.Name, &vf)));
}

bool mtsBIGSSVFController::Solve(vctDoubleVec & dq)
{
    nmrConstraintOptimizer::STATUS OptimizerStatus;
    OptimizerStatus = mtsVFController::Solve(dq);

    if (OptimizerStatus == nmrConstraintOptimizer::NMR_OK) {
      return true;
    }

    dq.SetAll(0);
    return false;
}

void mtsBIGSSVFController::PrintProblem()
{
  std::cout << "\n\nObjectiveMatrix:\n" << Optimizer.GetObjectiveMatrix() << std::endl;
  std::cout << "\nObjectiveVector:\n" << Optimizer.GetObjectiveVector() << std::endl;
  std::cout << "\nEqualityConstraintMatrix:\n" << Optimizer.GetEqConstraintMatrix() << std::endl;
  std::cout << "\nEqualityConstraintVector:\n" << Optimizer.GetEqConstraintVector() << std::endl;
  std::cout << "\nInequalityConstraintMatrix:\n" << Optimizer.GetIneqConstraintMatrix() << std::endl;
  std::cout << "\nInequalityConstraintVector:\n" << Optimizer.GetIneqConstraintVector() << std::endl;
}


void mtsBIGSSVFController::PrintSensors()
{
  std::cout << "Sensors: " << Sensors.size() << std::endl;

  std::map<std::string, prmSensorState*>::iterator itSen;
  for (itSen = Sensors.begin(); itSen != Sensors.end(); itSen++)
  {
    std::cout << itSen->second->HumanReadable() << std::endl;
  }
}

void mtsBIGSSVFController::PrintKinematics()
{
  std::cout << "Kinematics: " << Kinematics.size() << std::endl;

  std::map<std::string, prmKinematicsState*>::iterator itKin;
  for (itKin = Kinematics.begin(); itKin != Kinematics.end(); itKin++)
  {
    std::cout << itKin->second->HumanReadable() << std::endl;
  }
}

void mtsBIGSSVFController::PrintConstraints()
{
  std::cout << "VFs: " << VFMap.size() << std::endl;

  std::map<std::string, mtsVFBase*>::iterator itVF;
  for (itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
  {
    std::cout << itVF->second->Data->HumanReadable() << std::endl;
  }
}

