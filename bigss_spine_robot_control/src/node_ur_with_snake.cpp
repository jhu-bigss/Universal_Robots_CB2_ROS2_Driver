#include <QApplication>
#include "rclcpp/rclcpp.hpp"

// CISST Imports
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstVector/vctDynamicVector.h>
#include <cisst_ros2_bridge/mtsROSBridge.h>
#include <cisst_ros2_crtk/mts_ros_crtk_bridge.h>

// bigss_robotic_system Imports
#include <bigss_robotic_system/SeriallyAttachedRoboticSystemHelper.h>
#include <bigss_robotic_system/mtsSARobSysTask.h>
#include <bigss_robotic_system/SeriallyAttachedRoboticSystem.h>
#include <bigss_robotic_system/RobotInterfaceObject.h>

// ur_cb2_gui Imports
#include <ur_cb2_gui/RoboticSystemWidget.h>
#include <ur_cb2_gui/RoboticSystemInputsWidget.h>
#include <ur_cb2_gui/RoboticSystemConstraintsWidget.h>
#include <ur_cb2_gui/RoboticSystemStatusWidget.h>
#include <ur_cb2_gui/RoboticSystemControlWidget.h>


void SetupGUI(mtsComponentManager* manager, const std::string& task_name, RoboticSystemWidget& mainControlWidget);

int main(int argc, char * argv[])
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // Setup ROS2
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("ur_pol_clc");

    // Call to use CISST command line arguments
    cmnCommandLineOptions options;

    // 
    mtsComponentManager* manager = mtsManagerLocal::GetInstance();
    mts_ros_crtk_bridge* crtk_ros_bridge = new mts_ros_crtk_bridge("ur_robot_control_ctrk_bridge", ros_node);
    manager->AddComponent(crtk_ros_bridge);

    mtsROSBridge* ros_bridge = new mtsROSBridge("bridge", 50.0 * cmn_ms, ros_node);
    ros_bridge->PerformsSpin(true);
    manager->AddComponent(ros_bridge);

    // Qt User Interface
    QApplication app(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    RoboticSystemWidget main_control_widget;
    SetupGUI(manager, "SystemTask", main_control_widget);

    crtk_ros_bridge->Connect();

    main_control_widget.show();

    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

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