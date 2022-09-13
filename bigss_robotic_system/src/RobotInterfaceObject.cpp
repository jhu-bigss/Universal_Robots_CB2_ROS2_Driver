#include <bigss_robotic_system/RobotInterfaceObject.h>
#include <cisstVector.h>
#include <bigss_robotic_system/KinematicsObject.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include<bigss_robotic_system/SensorInterface.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>

//TODO: MoveCommandInterface, PeripheralCommandInterface, etc
//TODO: Handle multiple types of move command, not just joint velocity!


    RobotInterfaceObject::RobotInterfaceObject(const vctFrm3& frame_offset_to_base, const std::string& interface_name, int num_joints, const std::string& base_frame_name):
        frame_offset_to_base(frame_offset_to_base),
        interface_name(interface_name),
        num_joints(num_joints),
        base_frame_name(base_frame_name),
        KinematicsObject(interface_name, num_joints){ 
           SetName(interface_name);
        // m_measured_cp.SetReferenceFrame(base_frame_name); //TODO: doesn't work if done here. fix this and remove from derived
        // m_measured_cp.SetMovingFrame(interface_name);
    }

    // RobotInterfaceObject(const vctFrm3& frame_offset_to_base, const std::string& interface_name):
    // frame_offset_to_base(frame_offset_to_base),
    // interface_name(interface_name),
    // KinematicsObject(interface_name)
    // {}

    RobotInterfaceObject::~RobotInterfaceObject(){}

    void RobotInterfaceObject::MoveRobot(prmVelocityJointSet& command){} 

    void RobotInterfaceObject::UpdateSensors(){
        for(auto& sensor : this->sensor_interfaces){
            sensor->Update();
        }
    }
