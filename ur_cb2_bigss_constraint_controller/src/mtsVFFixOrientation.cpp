/*
Author: Henry Phalen
Adds term to constraint function that will result in joint inputs x to keep a given frame on 
the kinematic chain aligned to a specified orientation. Fills out the matrix A and vector b for a 
term to add to the constraint function:
argmin_x: ||Ax+b||
The frame on the kinematic chain can be specified arbitrarily by specifying the kinematics of the 
last joint that moves the frame of interest as well as any offset from that joint to the frame of interest
The target orientation is represented as a rotation from the base frame to the goal frame

Calculates the incremental rotation needed to more closely align with the goal orientation and then selects
joint inputs that will best achieve this incremental rotation

argmin_x: zeta * || Jw * x + invsk(R_goal*R^-1 - I) ||
zeta is a scaling factor to determine "importance" of constraint
Jw is the 'angular' jacobian at the point of interest, i.e. [angular_velocity = Jw * joint_velocity]
R is the orientation of the frame of interest relative to the base frame, R_goal is what you want this to be
invsk denotes the 'vec' operator, that does the opposite of the 'skew' operator
*/

#include <ur_cb2_bigss_constraint_controller/mtsVFFixOrientation.h>
#include <ur_cb2_bigss_constraint_controller/helper.h>

void mtsVFFixOrientation::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{ 
    mtsVFDataFixOrientation * data = reinterpret_cast<mtsVFDataFixOrientation*>(Data);

    // Calculate pose and jacobian at frame of interest (the frame you are trying to set the orientation of)
    // Note this jacobian is for all joints in the full system, columns associated with more distal joints are set to zero
    vctFrm3 frame_of_interest = data->kinematics_for_last_joint_that_moves_frame_of_interest->Frame * data->offset_from_kinematics_to_frame_of_interest;
    vctDoubleMat jacobian_of_interest = HTPChangeBIGSSJacobianToOffsetFromEEF(data -> kinematics_for_last_joint_that_moves_frame_of_interest->Jacobian,
        data -> offset_from_kinematics_to_frame_of_interest,
        data -> kinematics_for_last_joint_that_moves_frame_of_interest->Frame);
    vctDoubleMat system_angular_jacobian_at_frame_of_reference;
    ExtractAngularJacobianToArbitrarySize(jacobian_of_interest, system_angular_jacobian_at_frame_of_reference, data->num_joints_system);
    
    auto& current_orientation = frame_of_interest.Rotation();
    vct3 vect;
    InvSkew3x3MatTo3Vec((data->desired_orientation * current_orientation.Inverse())-vct3x3::Eye(), vect);
    // Fill in A matrix
    ObjectiveMatrixRef.Assign(data->Importance * system_angular_jacobian_at_frame_of_reference);
    // Fill in b vector
    ObjectiveVectorRef.Assign(data->Importance * vect);
}
