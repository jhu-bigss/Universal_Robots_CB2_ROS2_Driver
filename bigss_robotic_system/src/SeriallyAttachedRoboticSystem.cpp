/*
  Authors: Henry Phalen
*/

#include <bigss_robotic_system/SeriallyAttachedRoboticSystem.h>
#include <bigss_robotic_system/SeriallyAttachedRoboticSystemHelper.h>
#include <bigss_robotic_system/KinematicsObject.h>
#include <bigss_robotic_system/RobotInterfaceObject.h>
#include <bigss_robotic_system/SensorInterface.h>
// #include <cisstMultiTask/mtsManagerLocal.h>

/**
 * @brief Construct a new Serially Attached Robotic System. This can be used in the generalized controller task. 
 * 
 * @param system_name 
 * @param robot_interfaces vector of pointers to all robot interface objects
 * @param system_sensor_interfaces vector of pointers to all sensor interface objects that will be owned by the system itself (not those that contained robots own)
 * @details You can instantiate this class if you have a simple system, for more complicated systems, you can derive a class and override functions e.g. UpdateState()
 */
SeriallyAttachedRoboticSystem::SeriallyAttachedRoboticSystem(const std::string& system_name, const std::vector<RobotInterfaceObject*>& robot_interfaces, const std::vector<SensorInterface*>& system_sensor_interfaces):
    robot_interfaces(robot_interfaces),
    sensor_interfaces(system_sensor_interfaces),
    KinematicsObject(system_name)
{
    for(auto& robot: robot_interfaces){
        for (auto& sensor: robot->sensor_interfaces){
            sensor_interfaces.insert(sensor_interfaces.begin(),sensor); 
        }
        num_joints += robot->GetNumberOfJoints();
    }
    this->InitializeKinematicsStateSize();
    system_state.SetSize(num_joints);
    system_state.CartesianPosition().SetAll(0.0);
    system_state.JointPosition().SetSize(num_joints);
    system_state.JointPosition().SetAll(0.0);
    system_state.JointVelocityGoal().SetAll(0.0);
    goal_kinematics_state.Name=system_name+"Goal";

    system_measured_cp.SetReferenceFrame("base");
    system_measured_cp.SetMovingFrame(name);
}

SeriallyAttachedRoboticSystem::~SeriallyAttachedRoboticSystem(){};

// TODO? Being able to set up and configure a system from a configuration file would be a nice feature to add (see code below)
// void SeriallyAttachedRoboticSystem::SetupSystemFromConfigDir(std::string& path_to_config_dir){};

//TODO: I wanted systems to handle their own connections, but this could get messy so I didn't use the function
  // should this happen in main? Also needs to happen via config/JSON files, ask Anton
/*void SeriallyAttachedRoboticSystem::ConnectInterfaces(std::vector<std::string>& provided_task_names, std::vector<std::string>& provided_interface_names, 
    std::vector<std::string>& required_task_names, std::vector<std::string>& required_interface_names)
{
    mtsManagerLocal* component_manager = mtsManagerLocal::GetInstance();
    //TODO: Check size match
    for(size_t i=0; i<own_required_interfaces.size(); i++){
        component_manager->Connect(own_required_task_names[i], own_required_interfaces[i]->GetName(), provided_task_names[i], provided_interface_names[i]);
    } 
    for(size_t i=0; i<own_provided_interfaces.size(); i++){
        component_manager->Connect(required_task_names[i], required_interface_names[i], own_provided_task_names[i], own_provided_interfaces[i]->GetName());
    } 
}
*/

/**
 * @brief Tell all sensors (those owned by component robots and owned by the system itself) to update their values
 * 
 * @return true if all sensors update successfully, false and early if a sensor fails to update 
 */
bool SeriallyAttachedRoboticSystem::UpdateSensors(){
    //TODO? Could change to tell robots to update their own sensors and only update "owned" sensors?
    //TODO? Might not want to return early?
    for(auto& sensor : this->sensor_interfaces){
        if(!sensor->Update())
            {return false;}
    }
    return true;
}

/**
 * @brief Update the system's state, including commanding component robots to update their states first
 * 
 * @return true if component robots and system update state successfully, false and early if a component robot fails to update state
 * @details If you would like **system** sensors to impact the system's forward kinematics, 
 *   you can overwrite this UpdateStates function in a class derived from this one. 
 *   Just be sure that you always update sensors before updating states!
 */
bool SeriallyAttachedRoboticSystem::UpdateState(){
    if (!this->UpdateSensors()) // TODO? Could remove here and force separate call
        {return false;} 
    
    for(auto& robot: robot_interfaces)
    {
        if(!robot->UpdateState())
            {return false;}
    }

    this->UpdateSystemStatesFromRobotStates();
    return true;
}

/**
 * @brief Update the system state based on the serial connection of component robots
 * 
 */
void SeriallyAttachedRoboticSystem::UpdateSystemStatesFromRobotStates(){
    // TODO?: KDL or some other existing implementation
    // establish kinematic state with first (base) robot
    
    kinematics_state.JointState->SetValid(false);
    system_measured_cp.SetValid(false); // don't allow publishing data if it is bad
    
    kinematics_state.Jacobian.SetAll(0.0);
    
    RobotInterfaceObject* base_robot = robot_interfaces[0];
    kinematics_state.JointState->Position() = base_robot->kinematics_state.JointState->Position();
    kinematics_state.JointState->Velocity() = base_robot->kinematics_state.JointState->Velocity();
    kinematics_state.JointState->Effort() = base_robot->kinematics_state.JointState->Effort();
    kinematics_state.Frame = base_robot->frame_offset_to_base; // just do offset first, so you can adjust jacobian correctly
    vctDoubleMat offset_transformed_jacobian = ChangeFrameOfJacobian(base_robot->kinematics_state.Jacobian, kinematics_state.Frame);
    PlaceMatIntoMatAtIndex(offset_transformed_jacobian, kinematics_state.Jacobian, 0);
    
    kinematics_state.Frame = kinematics_state.Frame * base_robot->kinematics_state.Frame; // add on the robot's FK
    int next_index = base_robot->num_joints; //This keeps track of where to add Jacobian columns, TODO: implementation could be cleaned up
    // sequentially add on additional robots
    if (robot_interfaces.size()>1){
        for(size_t i = 1; i<robot_interfaces.size(); i++){
            RobotInterfaceObject* robot = robot_interfaces[i];
            ConcatenateTwoVctVec(vctDoubleVec(this->kinematics_state.JointState->Position()), robot->kinematics_state.JointState->Position(), this->kinematics_state.JointState->Position());
            ConcatenateTwoVctVec(vctDoubleVec(this->kinematics_state.JointState->Velocity()), robot->kinematics_state.JointState->Velocity(), this->kinematics_state.JointState->Velocity());
            ConcatenateTwoVctVec(vctDoubleVec(this->kinematics_state.JointState->Effort()), robot->kinematics_state.JointState->Effort(), this->kinematics_state.JointState->Effort());
            
            //TEST JACOBIAN 
            vctDoubleMat prior_jac(6,next_index);
            for( size_t i=0; i< next_index; i++){
                prior_jac.Column(i) = this->kinematics_state.Jacobian.Column(i); // getting the jacobian of all prior joints
            }
            //we want just the earlier parts of the jacobian
            vctDoubleMat transformed_prior_jac = HTPChangeBIGSSJacobianToOffsetFromEEF(this->kinematics_state.Jacobian, (robot->frame_offset_to_base * robot->kinematics_state.Frame), this->kinematics_state.Frame);
            PlaceMatIntoMatAtIndex(transformed_prior_jac, kinematics_state.Jacobian, 0);
            
            // END TEST
            this->kinematics_state.Frame = this->kinematics_state.Frame * robot->frame_offset_to_base; // just do offset first, so you can adjust jacobian correctly

            

            vctDoubleMat transformed_robot_jac = ChangeFrameOfJacobian(robot->kinematics_state.Jacobian, this->kinematics_state.Frame);
            PlaceMatIntoMatAtIndex(transformed_robot_jac, kinematics_state.Jacobian, next_index);

            next_index += robot->num_joints;    
            this->kinematics_state.Frame = this->kinematics_state.Frame * robot->kinematics_state.Frame; // add on the robot's FK
            


        }
    }     
    // TODO?: Inverse Jacobian not implemented to save computation - if you need it, just invert it yourself
    // TODO: Implement: cart vel, ang vel, prmRobotState

    this->kinematics_state.Name = this->name;
    ConvertFrm3ToR6(kinematics_state.Frame, system_state.CartesianPosition());
    system_state.JointPosition() = kinematics_state.JointState->Position();
    kinematics_state.JointState->SetValid(true);

    system_measured_cp.Position() = kinematics_state.Frame;
    system_measured_cp.SetValid(true); // needed so data publishes

    // TODO: may need additional system_state updates here
}

//TODO: Redundant, need to think about best way to implement this
void SeriallyAttachedRoboticSystem::MoveRobot(prmVelocityJointSet& system_command){ 
    Move(system_command);
}


/**
 * @brief Sends commands to each component robot to set its joint velocities
 * 
 * @param system_command All joint velocities for the entire system in order from base to last robot
 */
void SeriallyAttachedRoboticSystem::Move(prmVelocityJointSet& system_command){ 
    //TODO: Make not just velocities! Likely will need to move_velocity, move_position, etc. Make sure it matches crtk framework
    int begin_index = 0;
    for(auto& robot: this->robot_interfaces){
        prmVelocityJointSet robot_command;
        robot_command.SetSize(num_joints);
        int slice_length = robot->GetNumberOfJoints();
        SlicePrmVelocityJointSet(system_command, robot_command, begin_index, slice_length);
        begin_index += slice_length;
        robot->MoveRobot(robot_command);
    }
}

/**
 * @brief Sets velocity goals to zero for all robots in system
 * 
 */
void SeriallyAttachedRoboticSystem::StopMovement(){ 
    //TODO: better implementation, especially when using position, etc.
    system_state.JointVelocityGoal().SetAll(0.0);
    prmVelocityJointSet zero_command;
    zero_command.SetSize(num_joints);
    zero_command.SetGoal(system_state.JointVelocityGoal());
    this->Move(zero_command);
}
