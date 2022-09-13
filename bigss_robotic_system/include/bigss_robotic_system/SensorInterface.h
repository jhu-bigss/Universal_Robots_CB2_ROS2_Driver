#ifndef _BIGSSSensorInterface_h
#define _BIGSSSensorInterface_h


#include <cisstVector.h>
#include <cisstMultiTask/mtsComponent.h>
#include <sawConstraintController/prmSensorState.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsFunctionRead.h>

class SensorInterface: public mtsComponent{
    
    public:
    SensorInterface(const std::string& interface_name="SensorInterface");

    ~SensorInterface();
    virtual bool Update();

    std::string interface_name;
    prmSensorState sensor_state;   
    std::vector<mtsInterfaceRequired*> read_interfaces = {};
    std::vector<mtsFunctionRead*> read_functions = {};
    private:
    
    };

    #endif