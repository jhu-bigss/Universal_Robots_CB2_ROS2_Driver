#ifndef _BIGSSRobotInterface_h
#define _BIGSSRobotInterface_h

#include <cisstVector.h>
#include <cisstMultiTask/mtsComponent.h>
#include <bigss_robotic_system/KinematicsObject.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <bigss_robotic_system/SensorInterface.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>


//TODO: MoveCommandInterface, PeripheralCommandInterface, etc
//TODO: Handle multiple types of move command, not just joint velocity!

class RobotInterfaceObject: public KinematicsObject, public mtsComponent{
    
    public:
    
    RobotInterfaceObject(const vctFrm3& frame_offset_to_base=vctFrm3(), const std::string& interface_name="RobotInterfaceObject", 
        int num_joints=0, const std::string& base_frame_name="base");

    ~RobotInterfaceObject();

    virtual void MoveRobot(prmVelocityJointSet& command);
    void UpdateSensors();

    vctFrm3 frame_offset_to_base;
    const std::string interface_name;
    int num_joints;
    const std::string base_frame_name;
    std::vector<SensorInterface*> sensor_interfaces = {};
    std::vector<mtsInterfaceRequired*> command_interfaces = {};
    std::vector<mtsFunctionWrite*> command_functions = {};
    std::vector<mtsInterfaceRequired*> read_interfaces = {};
    std::vector<mtsFunctionRead*> read_functions = {};    
    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet m_base_offset_measured_cp;

    private:

};

#endif