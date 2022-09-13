/*
Author: Henry Phalen
Adds term to constraint function that will result in joint inputs x to drive the position of the end effector
to a specified point in 3D space. Used sequentially, this allows for trajectory following
Fills out the matrix A and vector b for a term to add to the constraint function:
argmin_x: ||Ax+b||

argmin_x: || Jp*x + v_des ||
Jp is the 'positional' jacobian at the point of interest, i.e. [linear_velocity = Jp * joint_velocity]
v_des is the desired incremental velocity of the end effector over the next time step
*/

#include <ur_cb2_bigss_constraint_controller/mtsVFFollowPositionTrajectory.h>

void mtsVFFollowPositionTrajectory::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    mtsVFDataFollowPositionTrajectory * data = reinterpret_cast<mtsVFDataFollowPositionTrajectory*>(Data);
    double dt = TickTime;
    vct3 p_to_goal = data->goal_kinematics->Frame.Translation() - data->current_kinematics->Frame.Translation();

    // Performs linear interpolation in case far from the goal point
    double T = p_to_goal.Norm() / data->goal_linear_velocity; // time to goal at desired linear velocity (s)
    double lambda = std::max(0.0, (T - dt) / T);
    vct3 p_interp_goal = data->goal_kinematics->Frame.Translation() - lambda * (p_to_goal);
  
    vct3 p_move = p_interp_goal - data->current_kinematics->Frame.Translation();
    if (Data->ObjectiveRows == 3)
    {
        // Fill A matrix
        ObjectiveMatrixRef.Assign(data->current_kinematics->Jacobian.Ref(3, data->current_kinematics->Jacobian.cols())); // extracts position rows of jacobian
        // Fill b vector
        ObjectiveVectorRef.Assign(p_move / dt); // this is the desired linear velocity
    }
    else
    {
        std::cerr << "[mtsVFFollowPositionTrajectory]: Incorrect number of Objective rows" << std::endl;
    }
}
