/*
Authors: Henry Phalen
*/

#ifndef _SERIALLYATTACHEDROBOTICSYSTEM_h
#define _SERIALLYATTACHEDROBOTICSYSTEM_h

#include <cisstVector.h>
#include <bigss_robotic_system/KinematicsObject.h>
#include <bigss_robotic_system/RobotInterfaceObject.h>
#include <bigss_robotic_system/SensorInterface.h>

#include <sawConstraintController/prmKinematicsState.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmRobotState.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

class SeriallyAttachedRoboticSystem: public KinematicsObject{
    
    public:

    SeriallyAttachedRoboticSystem(const std::string& system_name="System", const std::vector<RobotInterfaceObject*>& robot_interfaces = {}, const std::vector<SensorInterface*>& sensor_interfaces = {});

    ~SeriallyAttachedRoboticSystem();
    // TODO: might be worth just having a virtual SetupSystem, (called in the initializer?) and an optional SetupSystemFromConfigDir?
    void SetupSystemFromConfigDir(std::string& path_to_config_dir);
    //TODO: should this happen in main? Also needs to happen via config/JSON files, ask Anton
    void ConnectInterfaces(std::vector<std::string>& provided_task_names, std::vector<std::string>& provided_interface_names, 
        std::vector<std::string>& required_task_names, std::vector<std::string>& required_interface_names);
    bool UpdateSensors();
    virtual bool UpdateState();
    void UpdateSystemStatesFromRobotStates();
    void MoveRobot(prmVelocityJointSet& system_command);
    void Move(prmVelocityJointSet& system_command);
    void StopMovement();
    
    
    std::vector<RobotInterfaceObject*> robot_interfaces = {};
    std::vector<SensorInterface*> sensor_interfaces = {};
    prmRobotState system_state;
    prmKinematicsState goal_kinematics_state;
    prmPositionCartesianGet system_measured_cp;
    
    // TODO: these are to allow systems to self-connect, but needs work
    std::vector<mtsInterfaceProvided*> own_provided_interfaces = {};
    std::vector<std::string> own_provided_task_names = {};
    std::vector<mtsInterfaceRequired*> own_required_interfaces = {};
    std::vector<std::string> own_required_task_names = {};

    private:
    // std::vector<KinematicsObject*> kinematic_component_list;
    
};

#endif //_SERIALLYATTACHEDROBOTICSYSTEM_h