#include <bigss_robotic_system/SensorInterface.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsFunctionRead.h>

SensorInterface::SensorInterface(const std::string& interface_name):
    interface_name(interface_name)
    {
        SetName(interface_name); //this sets the mtsComponent name
    }

SensorInterface::~SensorInterface(){};

bool SensorInterface::Update(){}