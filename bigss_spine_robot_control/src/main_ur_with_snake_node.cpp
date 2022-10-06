#include <QApplication>
#include "rclcpp/rclcpp.hpp"

// CISST Imports
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstVector/vctDynamicVector.h>
#include <cisst_ros2_crtk/mts_ros_crtk_bridge_provided.h>

// bigss_robotic_system Imports
#include <bigss_robotic_system/SeriallyAttachedRoboticSystemHelper.h>
#include <bigss_robotic_system/mtsSARobSysTask.h>
#include <bigss_robotic_system/SeriallyAttachedRoboticSystem.h>
#include <bigss_robotic_system/RobotInterfaceObject.h>
#include <URInterface/URInterface.h>

#include <bigss_ur_gui/RoboticSystemWidget.h>
#include <bigss_ur_gui/RoboticSystemInputsWidget.h>
#include <bigss_ur_gui/RoboticSystemConstraintsWidget.h>
#include <bigss_ur_gui/RoboticSystemStatusWidget.h>
#include <bigss_ur_gui/RoboticSystemControlWidget.h>

#include <QApplication>
#include <QMainWindow>


void SetupGUI(mtsComponentManager* manager, const std::string& task_name, RoboticSystemWidget& mainControlWidget);

int main(int argc, char * argv[])
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);

    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // Setup ROS2
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("bigss_spine");

    // Call to use CISST command line arguments
    cmnCommandLineOptions options;

    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    // create a Qt user interface if needed
    QApplication app(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    mtsComponentManager* manager = mtsManagerLocal::GetInstance();

    // ROS 2 bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("spine_robot_control_crtk_bridge", ros_node);
    manager->AddComponent(crtk_bridge);


    // UR Interface
    vctFrm3 frame_offset_to_base;
    frame_offset_to_base.Identity(); 
    URInterface* ur_interface = new URInterface(frame_offset_to_base, "URInterface", 6, "base"); //TODO: defaults not working, must be a constructor/initializer thing
    manager->AddComponent(ur_interface);
    // manager->Connect(ur_interface->GetName(),"URInterface_required",robot_server.GetName(), "server");

    RoboticSystemWidget * mainControlWidget = new RoboticSystemWidget;
    // SetupGUI(manager, "system_task_test", *mainControlWidget);

    crtk_bridge->Connect();

    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    mainControlWidget->show();
    app.exec();

    // Stop all logs
    cmnLogger::Kill();

    // Stop ROS node
    rclcpp::shutdown();

    // Kill all components and perform cleanup
    manager->KillAllAndWait(5.0 * cmn_s);
    manager->Cleanup();

    return 0;
}

void SetupGUI(mtsComponentManager* manager, const std::string& task_name, RoboticSystemWidget& mainControlWidget){
    manager->AddComponent(mainControlWidget.inputs_); 
    manager->AddComponent(mainControlWidget.control_); 
    manager->AddComponent(mainControlWidget.constraints_); 
    manager->AddComponent(mainControlWidget.status_);
    manager->Connect(mainControlWidget.inputs_->GetName(), "userInterface", task_name, "userInterface");
    manager->Connect(mainControlWidget.control_->GetName(), "userInterface", task_name, "userInterface");
    manager->Connect(mainControlWidget.constraints_->GetName(), "userInterface", task_name, "userInterface");
    manager->Connect(mainControlWidget.status_->GetName(), "userInterface", task_name, "userInterface");
}
