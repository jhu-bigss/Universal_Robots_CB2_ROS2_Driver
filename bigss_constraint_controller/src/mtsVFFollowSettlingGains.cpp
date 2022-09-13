/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening, Henry Phalen
  Created on: 2014

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <bigss_constraint_controller/mtsVFFollowSettlingGains.h>
#include <cisstCommon/cmnUnits.h>
//! Updates co with virtual fixture data.
/*! FillInTableauRefs
    
    @brief Calculate joint angles that take the robot from the current
    position and orientation to a desired one.

    This is useful for having the arm follow a path,
    particularly one that leads the end effector slightly 
    at each instant to indicate the new destination.

    fill in refs
    min || J*dq - [v_T ; v_R] ||
    v_T = p_des - R_des * R^(-1)_cur * p_cur
    v_R = sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    J is the jacobian, p_des is the desired frame's translation,
    R_des is the desired frame's rotation, R^(-1)_cur is the inverse of the
    current frame's rotation, p_cur is the current frame's translation, A is
    an arbitrary vector, sk(A)^(-1) is the inverse of the skew matrix of A
    

   The goal is constructing the dx parameter which contains:
   [x, y, z, rx*theta, ry*theta, rz*theta]
   This is the same xdot format output by J*qdot
   that way we try to make xdot - J*qdot == [0,0,0,0,0,0].transpose()
   which means that you've found the joint angles that
   will put you at exactly the desired position and rotation

   This function was changed from the original to allow straight line cartesian movement towards a goal, but then settle into the goal within
   a threshold using PID gains.
   //TODO: finish implementation and clean up
   //TODO: make things 'setable' like vel_norms, k_periods (or just threshold), both PIDs,
*/
void mtsVFFollowSettlingGains::FillInTableauRefs(const CONTROLLERMODE mode,
    const double TickTime)
{
    CurrentKinematics = Kinematics.at(0);
    mtsVFDataFollowGains * GainsData = (mtsVFDataFollowGains*)(Data);
    vct3 gains = GainsData->gains;

    vctFrm3 measured_cp_Frm3 = Kinematics.at(0)->Frame;
    vctFrm3 target_cp_Frm3 = Kinematics.at(1)->Frame;
    double dt = TickTime; 
    double pos_vel_norm = 10.0*cmn_mm; //m/s 
    double ang_vel_norm = 1.0*cmnPI_180; //rad/s
    double pos_k_periods = 20.0;
    double rot_k_periods = 5.0;

    double pos_tresh = pos_k_periods*pos_vel_norm*dt; //m
    double rot_thresh = rot_k_periods*ang_vel_norm*dt;; //rad

    vctRot3 R_to_goal = measured_cp_Frm3.Rotation().Inverse() * target_cp_Frm3.Rotation();
    vctAxAnRot3 R_to_goal_ax_ang(R_to_goal);
    vct3 p_to_goal = target_cp_Frm3.Translation() - measured_cp_Frm3.Translation();

    double T_rot = std::abs(R_to_goal_ax_ang.Angle())/ang_vel_norm;
    double T_pos = p_to_goal.Norm() /pos_vel_norm;
    double T = std::max(T_rot,T_pos);
    double lambda = std::max(0.0, (T-dt)/T);

    double angle_current = R_to_goal_ax_ang.Angle();
    vct3 rot_current = R_to_goal_ax_ang.Angle() * R_to_goal_ax_ang.Axis();

    vct3 pos_current = p_to_goal;
    if (first_iter){ //handle first time 
        pos_last = pos_current;
        rot_last = rot_current;
        first_iter = false;
    }

    vctRot3 R_interp_goal;
    vct3 p_interp_goal;
    vctDouble3x3 identity3x3 = vctDouble3x3::Eye();

    if (R_to_goal_ax_ang.Angle() > rot_thresh){
        R_interp_goal = target_cp_Frm3.Rotation() * vctRot3(vctAxAnRot3(R_to_goal_ax_ang.Axis(),R_to_goal_ax_ang.Angle()*-lambda));
        // std::cout << "Rot normal, R_interp_goal: " << R_interp_goal <<  std::endl;
    }
    else{
        rot_sum += rot_current;
        double kP_rot = gains(0); //TODO: implement and check D and I
        double kI_rot = 0.0;
        double kD_rot = 0.0;
        vctDouble3 rot_ctrl = kP_rot * identity3x3 * vctDouble3(rot_current) + kI_rot * identity3x3 * vctDouble3(rot_sum) + kD_rot * identity3x3 * vctDouble3((rot_current-rot_last));
        if (!rot_ctrl.AlmostEqual(vctDouble3(0.0))){ // prevents divide by zero error internal to creation of rotation
            R_interp_goal = measured_cp_Frm3.Rotation() * vctRot3(vctAxAnRot3(rot_ctrl.Normalized(),rot_ctrl.Norm()));

        }else{
            R_interp_goal = measured_cp_Frm3.Rotation();
        }
        // std::cout << "Rot settle, R_interp_goal: " << R_interp_goal <<  std::endl;
    }

    if (p_to_goal.Norm() > pos_tresh){
        p_interp_goal = target_cp_Frm3.Translation()-lambda*(p_to_goal);
        // std::cout << "Pos normal, p_interp_goal: " << p_interp_goal <<  std::endl;
    }else{
        pos_sum += pos_current;
        double kP_pos = gains(1); //TODO make inputs, also force between 0 and 1.
        double kI_pos = 0.0;
        double kD_pos = 0.0;
        vctDouble3 pos_ctrl = kP_pos * identity3x3 * vctDouble3(pos_current) + kI_pos * identity3x3 * vctDouble3(pos_sum) + kD_pos * identity3x3 * vctDouble3((pos_current-pos_last));
        p_interp_goal = measured_cp_Frm3.Translation()+pos_ctrl;
        // std::cout << "Pos settle, p_interp_goal: " << p_interp_goal <<  std::endl;

    }

    //TODO: cleanup implementation
    vctFrm3 interp_goal(R_interp_goal,p_interp_goal);
    vctFrm3 moveFrm3 = measured_cp_Frm3.Inverse() * interp_goal;
    vctDoubleVec moveR6(6);

    std::copy(moveFrm3.Translation().begin(),moveFrm3.Translation().end(), moveR6.begin());
    vctRodRot3 frame_rodrot(moveFrm3.Rotation(),false);
    std::copy(frame_rodrot.begin(),frame_rodrot.end(), moveR6.begin()+3);

    vctDouble3 w_move;
    std::copy(moveR6.begin()+3, moveR6.end(), w_move.begin());
    vct3 w_move_corrected_frame = measured_cp_Frm3.Rotation()*w_move;
    vctDouble3 v_move = interp_goal.Translation() - measured_cp_Frm3.Translation();

    vctDoubleVec dx(6);
    std::copy(v_move.begin(), v_move.end(),dx.begin());
    std::copy(w_move_corrected_frame.begin(), w_move_corrected_frame.end(),dx.begin()+3);
    dx /= dt;
    pos_last = pos_current;
    rot_last = rot_current;

    if(Data->ObjectiveRows == 3)
    {
        ObjectiveVectorRef.Assign(dx.Ref(3,0));
        ObjectiveMatrixRef.Assign(CurrentKinematics->Jacobian.Ref(3,CurrentKinematics->Jacobian.cols()));
    }
    else if(Data->ObjectiveRows == 6)
    {
        // Constructing the dx parameter which contains:
        // [x, y, z, rx*theta, ry*theta, rz*theta]
        // This is the same xdot format output by J*qdot
        // that way we try to make xdot - J*qdot == [0,0,0,0,0,0].transpose()
        // which means that you've found the joint angles that
        // will put you at exactly the desired position and rotation
        ObjectiveVectorRef.Assign(dx);
        ObjectiveMatrixRef.Assign(CurrentKinematics->Jacobian);

        //std::cout << "dx: " << dx << std::endl;
    }
    else{
        std::cerr << "Incorrect number of Objective rows" << std::endl;
    }

}

void mtsVFFollowSettlingGains::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    if(mode == JVEL)
    {
        //TickTime*A*dq >= TickTime*b
        ObjectiveMatrixRef.Assign(ObjectiveMatrixRef*TickTime);
        ObjectiveVectorRef.Assign(ObjectiveVectorRef*TickTime);
    }
}

mtsVFDataFollowGains::mtsVFDataFollowGains(void):
      mtsVFDataBase()
    , gains(vct3(0.0, 0.0, 0.0))
    , weights()
{}

mtsVFDataFollowGains::mtsVFDataFollowGains(const mtsVFDataFollowGains & other):
      mtsVFDataBase(other)
    , gains(other.gains)
    , weights(other.weights)
{}

mtsVFDataFollowGains::~mtsVFDataFollowGains(void) {}

std::string mtsVFDataFollowGains::HumanReadable(void) const {
    std::stringstream description__cdg;
    description__cdg << "mtsVFDataFollowGains" << std::endl;
    return description__cdg.str();
}
