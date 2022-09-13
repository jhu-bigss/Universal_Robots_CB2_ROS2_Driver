

#include <sawConstraintController/prmKinematicsState.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstVector.h>

#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFConfig.h>
#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFController.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFStayOnAxis.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFollowPositionTrajectory.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFFixOrientation.h>
#include <ur_cb2_bigss_constraint_controller/mtsBIGSSVFRCM.h>
#include <ur_cb2_bigss_constraint_controller/mtsVFJointPenalty.h>

// TODO: Need a large-scale simplification of this, constraints really should be a single type and configure themselves, etc.

mtsBIGSSVFConfig::mtsBIGSSVFConfig(const std::string &config_dir, int num_joints) // TODO: from config
    : num_joints(num_joints)
{
    // TODO: Allow for multiples of same type of constraint (this could be taken care of in larger simplification effort)
    VF_follow_data = mtsVFDataFollowGains();
    VF_follow_position_trajectory_data = mtsVFDataFollowPositionTrajectory();
    VF_joint_vel_limits_data = mtsVFDataJointLimits();
    VF_absolute_joint_limits_data = mtsVFDataAbsoluteJointLimits();
    VF_rcm_data = mtsBIGSSVFDataRCM();
    VF_joint_penalty_data = mtsVFDataJointPenalty();
    VF_stay_on_axis_data = mtsVFDataStayOnAxis();
    VF_fix_orientation_data = mtsVFDataFixOrientation();
}

mtsBIGSSVFConfig::~mtsBIGSSVFConfig() {}

void mtsBIGSSVFConfig::SetAllActiveVFs(mtsBIGSSVFController &controller)
{
    // TODO?: configured check could go here, don't want to activate something never configured
    controller.DeactivateAll();

    if (VF_follow_active)
    {
        controller.AddVFFollowSettlingGains(VF_follow_data);
        for (auto &kin : VF_follow_kins)
        {
            controller.SetKinematics(*kin);
        }
        controller.ActivateVF(VF_follow_data.Name);
    }

    if (VF_follow_position_trajectory_active)
    {
        controller.AddVFFollowPositionTrajectory(VF_follow_position_trajectory_data);
        controller.ActivateVF(VF_follow_position_trajectory_data.Name);
    }

    if (VF_stay_on_axis_active)
    {
        controller.AddVFStayOnAxis(VF_stay_on_axis_data);
        controller.ActivateVF(VF_stay_on_axis_data.Name);
    }
    if (VF_fix_orientation_data_active)
    {
        controller.AddVFFixOrientation(VF_fix_orientation_data);
        controller.ActivateVF(VF_fix_orientation_data.Name);
    }
    if (VF_joint_vel_limits_active)
    {
        controller.AddVFJointVelLimits(VF_joint_vel_limits_data);
        controller.ActivateVF(VF_joint_vel_limits_data.Name);
    }
    if (VF_absolute_joint_limits_active)
    {
        controller.AddVFAbsoluteJointLimits(VF_absolute_joint_limits_data);
        for (auto &kin : VF_absolute_joint_limits_kins)
        {
            controller.SetKinematics(*kin);
        }
        controller.ActivateVF(VF_absolute_joint_limits_data.Name);
    }
    if (VF_rcm_active)
    {
        controller.AddVFRCM(VF_rcm_data);
        controller.ActivateVF(VF_rcm_data.Name);
    }
    if (VF_joint_penalty_active)
    {
        controller.AddVFJointPenalty(VF_joint_penalty_data);
        controller.ActivateVF(VF_joint_penalty_data.Name);
    }
}

// TODO: Most of the below need to be settable in a constructor for the data somehow, perhaps removed from here entirely?
// The old way was too scattered to be clear though, and just consisted of declaring things in a hard-coded way within the implementation of a controller

void mtsBIGSSVFConfig::SetVFDataFollowGains(bool active, prmKinematicsState &measured_kins, prmKinematicsState &target_kins, vctDoubleVec gains, const std::string &name)
{
    VF_follow_data.Name = name;
    VF_follow_data.ObjectiveRows = 6;
    VF_follow_data.KinNames.clear();
    VF_follow_data.KinNames.push_back(measured_kins.Name);
    VF_follow_data.KinNames.push_back(target_kins.Name);
    VF_follow_kins.push_back(&measured_kins);
    VF_follow_kins.push_back(&target_kins);
    VF_follow_data.gains = gains;
    VF_follow_active = active;
    VF_follow_configured = true;
}

void mtsBIGSSVFConfig::SetVFDataFollowGainsFromConfig(const std::string &filename, prmKinematicsState &measured_kins, prmKinematicsState &target_kins)
{
    // TODO: Remove kinematics, do all from name in config file string matching
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "go_to_goal_straight_line_cartesian")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vctDoubleVec gains;
            ReadVctDoubleVecFromConfig(constraint["gains"], gains);

            SetVFDataFollowGains(active, measured_kins, target_kins, gains, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFDataFollowPositionTrajectory(bool active, prmKinematicsState &current_kinematics, prmKinematicsState &goal_kinematics, double goal_linear_velocity, const std::string &name)
{
    VF_follow_position_trajectory_data.Name = name;
    VF_follow_position_trajectory_data.ObjectiveRows = 3;
    VF_follow_position_trajectory_data.current_kinematics = &current_kinematics;
    VF_follow_position_trajectory_data.goal_kinematics = &goal_kinematics;
    VF_follow_position_trajectory_data.goal_linear_velocity = goal_linear_velocity;
    VF_follow_position_trajectory_active = active;
    VF_follow_position_trajectory_configured = true;
}

void mtsBIGSSVFConfig::SetVFDataFollowPositionTrajectoryFromConfig(const std::string &filename, prmKinematicsState &current_kinematics, prmKinematicsState &goal_kinematics)
{
    // TODO: Remove kinematics, do all from name in config file string matching
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "follow_position_trajectory")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            double goal_linear_velocity = constraint["goal_linear_velocity_m/s"].asDouble();
            SetVFDataFollowPositionTrajectory(active, current_kinematics, goal_kinematics, goal_linear_velocity, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFStayOnAxis(bool active, prmKinematicsState &kinematics_for_last_joint_that_moves_frame_of_interest, vctFrm3 &offset_from_kinematics_to_frame_of_interest, vct3 &point_on_desired_axis, vct3 &desired_axis, double gain, const std::string &name)
{
    VF_stay_on_axis_data.Name = name;
    VF_stay_on_axis_data.ObjectiveRows = 3;
    VF_stay_on_axis_data.kinematics_for_last_joint_that_moves_frame_of_interest = &kinematics_for_last_joint_that_moves_frame_of_interest;
    VF_stay_on_axis_data.offset_from_kinematics_to_frame_of_interest = offset_from_kinematics_to_frame_of_interest;
    VF_stay_on_axis_data.point_on_desired_axis = point_on_desired_axis;
    VF_stay_on_axis_data.desired_axis = desired_axis;
    VF_stay_on_axis_data.num_joints_system = num_joints;
    VF_stay_on_axis_data.Importance = gain;
    VF_stay_on_axis_active = active;
    VF_stay_on_axis_configured = true;
    std::cout << VF_stay_on_axis_data << std::endl;
}

void mtsBIGSSVFConfig::SetVFStayOnAxisFromConfig(const std::string &filename, prmKinematicsState &kinematics_for_last_joint_that_moves_frame_of_interest, vctFrm3 &offset_from_kinematics_to_frame_of_interest)
{
    // TODO: Remove kinematics, do all from name in config file string matching, same with offset
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "stay_on_axis")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vct3 desired_axis;
            ReadVct3FromConfig(constraint["desired_axis"], desired_axis);
            vct3 point_on_desired_axis;
            ReadVct3FromConfig(constraint["point_on_desired_axis"], point_on_desired_axis);
            double gain = constraint["gain"].asDouble();
            SetVFStayOnAxis(active, kinematics_for_last_joint_that_moves_frame_of_interest, offset_from_kinematics_to_frame_of_interest, point_on_desired_axis, desired_axis, gain, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFFixOrientation(bool active, prmKinematicsState &kinematics_for_last_joint_that_moves_frame_of_interest, vctFrm3 &offset_from_kinematics_to_frame_of_interest, vctRot3 desired_orientation, double gain, const std::string &name)
{
    VF_fix_orientation_data.Name = name;
    VF_fix_orientation_data.ObjectiveRows = 3;
    VF_fix_orientation_data.kinematics_for_last_joint_that_moves_frame_of_interest = &kinematics_for_last_joint_that_moves_frame_of_interest;
    VF_fix_orientation_data.offset_from_kinematics_to_frame_of_interest = offset_from_kinematics_to_frame_of_interest;
    VF_fix_orientation_data.desired_orientation = desired_orientation;
    VF_fix_orientation_data.num_joints_system = num_joints;
    VF_fix_orientation_data.Importance = gain;

    VF_fix_orientation_data_active = active;
    VF_fix_orientation_data_configured = true;
}

void mtsBIGSSVFConfig::SetVFFixOrientationFromConfig(const std::string &filename, prmKinematicsState &kinematics_for_last_joint_that_moves_frame_of_interest, vctFrm3 &offset_from_kinematics_to_frame_of_interest)
{
    // TODO: Remove kinematics, do all from name in config file string matching, same with offset
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "fix_orientation")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vctRot3 desired_orientation;
            cmnDataJSON<vctRot3>::DeSerializeText(desired_orientation, constraint["desired_orientation"]);
            double gain = constraint["gain"].asDouble();
            SetVFFixOrientation(active, kinematics_for_last_joint_that_moves_frame_of_interest, offset_from_kinematics_to_frame_of_interest, desired_orientation, gain, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFJointVelLimits(bool active, vctDoubleVec &LowerVelLimits, vctDoubleVec &UpperVelLimits, const std::string &name)
{
    VF_joint_vel_limits_data.Name = name;
    VF_joint_vel_limits_data.IneqConstraintRows = 2 * num_joints;
    VF_joint_vel_limits_data.LowerLimits = LowerVelLimits;
    VF_joint_vel_limits_data.UpperLimits = UpperVelLimits;
    VF_joint_vel_limits_active = active;
    VF_joint_vel_limits_configured = true;
}
void mtsBIGSSVFConfig::SetVFJointVelLimitsFromConfig(const std::string &filename)
{
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "joint_velocity_limits")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vctDoubleVec lower_limits;
            ReadVctDoubleVecFromConfig(constraint["lower_limits"], lower_limits);
            vctDoubleVec upper_limits;
            ReadVctDoubleVecFromConfig(constraint["upper_limits"], upper_limits);

            SetVFJointVelLimits(active, lower_limits, upper_limits, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFRCM(bool active, prmKinematicsState &kinematics_for_last_joint_that_moves_closest_point, vctFrm3 &offset_from_kinematics_to_frame_of_interest, vct3 &RCM_point, const std::string &name)
{
    VF_rcm_data.Name = name;
    VF_rcm_data.IneqConstraintRows = num_joints;
    VF_rcm_data.kinematics_for_last_joint_that_moves_closest_point = &kinematics_for_last_joint_that_moves_closest_point;
    VF_rcm_data.offset_from_kinematics_to_frame_of_interest = offset_from_kinematics_to_frame_of_interest;
    VF_rcm_data.RCM_point = RCM_point;
    VF_rcm_active = active;
    VF_rcm_configured = true;
}

void mtsBIGSSVFConfig::SetVFRCMFromConfig(const std::string &filename, prmKinematicsState &kinematics_for_last_joint_that_moves_closest_point, vctFrm3 &offset_from_kinematics_to_frame_of_interest)
{
    // TODO: Remove kinematics, do all from name in config file string matching, same with offset
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "RCM")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vct3 RCM_point;
            ReadVct3FromConfig(constraint["RCM_point"], RCM_point);
            SetVFRCM(active, kinematics_for_last_joint_that_moves_closest_point, offset_from_kinematics_to_frame_of_interest, RCM_point, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFAbsoluteJointLimits(bool active, vctDoubleVec &LowerLimits, vctDoubleVec &UpperLimits, prmKinematicsState &current_kinematics, const std::string &name)
{
    VF_absolute_joint_limits_data.Name = name;
    VF_absolute_joint_limits_data.IneqConstraintRows = 2 * num_joints;
    VF_absolute_joint_limits_data.KinNames.clear();
    VF_absolute_joint_limits_data.KinNames.push_back(current_kinematics.Name);
    VF_absolute_joint_limits_kins.push_back(&current_kinematics);
    VF_absolute_joint_limits_data.LowerLimits = LowerLimits;
    VF_absolute_joint_limits_data.UpperLimits = UpperLimits;
    VF_absolute_joint_limits_active = active;
    VF_absolute_joint_limits_configured = true;
}

void mtsBIGSSVFConfig::SetVFAbsoluteJointLimitsFromConfig(const std::string &filename, prmKinematicsState &current_kinematics)
{
    // TODO: Remove kinematics, do all from name in config file string matching
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "absolute_joint_limits")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vctDoubleVec lower_limits;
            ReadVctDoubleVecFromConfig(constraint["lower_limits"], lower_limits);
            vctDoubleVec upper_limits;
            ReadVctDoubleVecFromConfig(constraint["upper_limits"], upper_limits);
            SetVFAbsoluteJointLimits(active, lower_limits, upper_limits, current_kinematics, name);
        }
    }
}

void mtsBIGSSVFConfig::SetVFJointPenalty(bool active, vctDoubleVec &joint_penalty_weights, const std::string &name)
{
    VF_joint_penalty_data.Name = name;
    VF_joint_penalty_data.ObjectiveRows = joint_penalty_weights.size(); // TODO?: num_joints?
    VF_joint_penalty_data.JointPenalty = joint_penalty_weights;
    VF_joint_penalty_active = active;
    VF_joint_penalty_configured = true;
}

void mtsBIGSSVFConfig::SetVFJointPenaltyFromConfig(const std::string &filename)
{
    Json::Value constraint_list;
    GetConstraintListFromConfigFile(filename, constraint_list);
    for (auto &constraint : constraint_list)
    {
        if (constraint["type"] == "joint_penalty")
        {
            bool active = constraint["active"].asBool();
            std::string name = constraint["name"].asString();
            vctDoubleVec joint_penalty_weights;
            ReadVctDoubleVecFromConfig(constraint["joint_penalty_weights"], joint_penalty_weights);
            SetVFJointPenalty(active, joint_penalty_weights, name);
        }
    }
}

void mtsBIGSSVFConfig::ReadVct3FromConfig(Json::Value &vct3_as_json_value, vct3 &output)
{
    // TODO: size checks / array check, etc.
    output[0] = vct3_as_json_value[0].asDouble();
    output[1] = vct3_as_json_value[1].asDouble();
    output[2] = vct3_as_json_value[2].asDouble();
}

void mtsBIGSSVFConfig::ReadVctDoubleVecFromConfig(Json::Value &vctDoubleVec_as_json_value, vctDoubleVec &output)
{
    // TODO: size checks / array check, etc.
    int size = vctDoubleVec_as_json_value.size();
    output.SetSize(size);
    for (auto i = 0; i < size; i++)
    {
        output[i] = vctDoubleVec_as_json_value[i].asDouble();
    }
}

void mtsBIGSSVFConfig::GetConstraintListFromConfigFile(const std::string &filename, Json::Value &constraints_list)
{
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());
    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    jsonReader.parse(jsonStream, jsonConfig);
    constraints_list = jsonConfig["constraints"];
    jsonStream.close();
    // std::cout << "constraint_list: " << constraints_list << std::endl;
}