/*
Author: Henry Phalen
Adds term to constraint function that will result in joint inputs x to keep a specified point on 
the kinematic chain on a line/axis defined in 3D space. Fills out the matrix A and vector b for a 
term to add to the constraint function:
argmin_x: ||Ax+b||
The point on the kinematic chain can be specified arbitrarily by specifying the kinematics of the 
last joint that moves the point of interest as well as any offset from that joint to the point of interest
The line is defined in 3D space by a point on the line and a direction vector

Minimizes the cross product between the direction vector of the axis you are trying to stay on and
the vector between a point on that axis and the new position of the point after the input joint values

argmin_x: zeta * || [-sk(v_axis) * Jp] x + sk(v_axis)*(p_curr-p_axis) ||
zeta is a scaling factor to determine "importance" of constraint
Jp is the 'positional' jacobian at the point of interest, i.e. [linear_velocity = Jp * joint_velocity]
p_curr is the current position of the point in 3D space
p_axis and v_axis define a line/axis in 3D space using a point p and direction vector v
*/

#include <bigss_constraint_controller/mtsVFStayOnAxis.h>
#include <bigss_constraint_controller/helper.h>

void mtsVFStayOnAxis::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{ 
    mtsVFDataStayOnAxis * data = reinterpret_cast<mtsVFDataStayOnAxis*>(Data);

    // Calculate pose and jacobian at point of interest (the point you are trying to get to stay on the axis)
    // Note this jacobian is for all joints in the full system, columns associated with more distal joints are set to zero
    vctFrm3 frame_of_interest = data->kinematics_for_last_joint_that_moves_frame_of_interest->Frame * data->offset_from_kinematics_to_frame_of_interest;
    vctDoubleMat jacobian_of_interest = HTPChangeBIGSSJacobianToOffsetFromEEF(data -> kinematics_for_last_joint_that_moves_frame_of_interest->Jacobian,
        data -> offset_from_kinematics_to_frame_of_interest,
        data -> kinematics_for_last_joint_that_moves_frame_of_interest->Frame);
    vctDoubleMat system_positional_jacobian_at_frame_of_reference;
    ExtractPositionalJacobianToArbitrarySize(jacobian_of_interest, system_positional_jacobian_at_frame_of_reference, data->num_joints_system);
    
    auto& current_pos = frame_of_interest.Translation();
    vctDoubleMat skew_ax;
    Skew3VecTo3x3Mat(data->desired_axis, skew_ax);
    // Fill in A matrix
    ObjectiveMatrixRef.Assign(-1.0 * data->Importance * skew_ax * system_positional_jacobian_at_frame_of_reference);
    // Fill in b vector
    ObjectiveVectorRef.Assign( data->Importance * skew_ax * vctDoubleVec(current_pos - data->point_on_desired_axis));
}
