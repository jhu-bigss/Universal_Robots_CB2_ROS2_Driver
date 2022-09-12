#ifndef MTSBIGSSVFCONFIG_H
#define MTSBIGSSVFCONFIG_H

#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstVector.h>
#include <sawConstraintController/mtsVFController.h>
#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFController.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFStayOnAxis.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFixOrientation.h>
#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFRCM.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFJointPenalty.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFollowPositionTrajectory.h>

class mtsBIGSSVFConfig
{

public:
    int num_joints;
    mtsVFDataFollowGains VF_follow_data;
    bool VF_follow_active = false;
    bool VF_follow_configured = false;
    std::vector<prmKinematicsState *> VF_follow_kins;

    mtsVFDataFollowPositionTrajectory VF_follow_position_trajectory_data;
    bool VF_follow_position_trajectory_active = false;
    bool VF_follow_position_trajectory_configured = false;

    mtsVFDataStayOnAxis VF_stay_on_axis_data;
    bool VF_stay_on_axis_active = false;
    bool VF_stay_on_axis_configured = false;

    mtsVFDataJointLimits VF_joint_vel_limits_data;
    bool VF_joint_vel_limits_active = false;
    bool VF_joint_vel_limits_configured = false;
    std::vector<prmKinematicsState *> VF_joint_vel_limits_kins;

    mtsVFDataAbsoluteJointLimits VF_absolute_joint_limits_data;
    bool VF_absolute_joint_limits_active = false;
    bool VF_absolute_joint_limits_configured = false;
    std::vector<prmKinematicsState *> VF_absolute_joint_limits_kins;

    mtsBIGSSVFDataRCM VF_rcm_data;
    bool VF_rcm_active = false;
    bool VF_rcm_configured = false;
    std::vector<prmKinematicsState *> VF_rcm_kins;

    mtsVFDataJointPenalty VF_joint_penalty_data;
    bool VF_joint_penalty_active = false;
    bool VF_joint_penalty_configured = false;

    mtsVFDataFixOrientation VF_fix_orientation_data;
    bool VF_fix_orientation_data_active = false;
    bool VF_fix_orientation_data_configured = false;

    // TODO: Configure everything just by passing this file into the constructor. Major simplifications can be made here
    mtsBIGSSVFConfig(const std::string &config_dir = "", int num_joints = 0);
    ~mtsBIGSSVFConfig();

    void SetAllActiveVFs(mtsBIGSSVFController &controller);

    void SetVFDataFollowGains(bool active, prmKinematicsState &measured_kins, prmKinematicsState &target_kins, vctDoubleVec gains, const std::string &name = "FOLLOW");
    void SetVFStayOnAxis(bool active, prmKinematicsState &kinematics_for_last_joint_that_moves_frame_of_interest, vctFrm3 &offset_from_kinematics_to_frame_of_interest, vct3 &point_on_desired_axis, vct3 &desired_axis, double gain, const std::string &name = "STAY_ON_AXIS");
    void SetVFJointVelLimits(bool active, vctDoubleVec &LowerVelLimits, vctDoubleVec &UpperVelLimits, const std::string &name = "JOINT_VEL_LIMITS");
    void SetVFRCM(bool active, prmKinematicsState &kinematics_for_last_joint_that_moves_closest_point, vctFrm3 &offset_from_kinematics_to_frame_of_interest, vct3 &RCM_point, const std::string &name = "RCM");
    void SetVFAbsoluteJointLimits(bool active, vctDoubleVec &LowerLimits, vctDoubleVec &UpperLimits, prmKinematicsState &current_kins, const std::string &name = "ABSOLUTE_LIMITS");
    void SetVFJointPenalty(bool active, vctDoubleVec &JointPenalties, const std::string &name = "JOINT_PENALTY");

    void SetVFFixOrientation(bool active, prmKinematicsState &kinematics_for_last_joint_that_moves_frame_of_interest, vctFrm3 &offset_from_kinematics_to_frame_of_interest, vctRot3 desired_orientation, double gain, const std::string &name = "FIX_ORIENTATION");

    void ReadVct3FromConfig(Json::Value &vct3_as_json_value, vct3 &output);
    void ReadVctDoubleVecFromConfig(Json::Value &vctDoubleVec_as_json_value, vctDoubleVec &output);
    void GetConstraintListFromConfigFile(const std::string &filename, Json::Value &constraints_list);

    void SetVFJointPenaltyFromConfig(const std::string &filename);

    void SetVFAbsoluteJointLimitsFromConfig(const std::string &filename, prmKinematicsState &current_kinematics);

    void SetVFJointVelLimitsFromConfig(const std::string &filename);

    void SetVFDataFollowGainsFromConfig(const std::string &filename, prmKinematicsState &measured_kins, prmKinematicsState &target_kins);

    void SetVFRCMFromConfig(const std::string &filename, prmKinematicsState &kinematics_for_last_joint_that_moves_closest_point, vctFrm3 &offset_from_kinematics_to_frame_of_interest);

    void SetVFStayOnAxisFromConfig(const std::string &filename, prmKinematicsState &kinematics_for_last_joint_that_moves_closest_point, vctFrm3 &offset_from_kinematics_to_frame_of_interest);

    void SetVFFixOrientationFromConfig(const std::string &filename, prmKinematicsState &kinematics_for_last_joint_that_moves_closest_point, vctFrm3 &offset_from_kinematics_to_frame_of_interest);

    void SetVFDataFollowPositionTrajectory(bool active, prmKinematicsState &current_kinematics, prmKinematicsState &goal_kinematics, double goal_linear_velocity, const std::string &name = "FOLLOW_POS_TRAJ");
    void SetVFDataFollowPositionTrajectoryFromConfig(const std::string &filename, prmKinematicsState &current_kinematics, prmKinematicsState &goal_kinematics);
};

#endif // MTSBIGSSVFCONFIG_H
