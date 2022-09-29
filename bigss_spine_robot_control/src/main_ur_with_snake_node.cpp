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

#include <QApplication>
#include <QMainWindow>

int main(int argc, char * argv[])
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);

    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // Setup ROS2
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("ur_pol_clc");

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
    QApplication * app = new QApplication(argc, argv);
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

    // crtk_bridge->Connect();

    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    app->exec();

    // Stop all logs
    cmnLogger::Kill();

    // Stop ROS node
    rclcpp::shutdown();

    // Kill all components and perform cleanup
    manager->KillAllAndWait(5.0 * cmn_s);
    manager->Cleanup();

    return 0;
}