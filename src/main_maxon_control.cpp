#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>

#include <cisst_ros2_bridge/mtsROSBridge.h>
#include <cisst_ros2_crtk/mts_ros_crtk_bridge_provided.h>

#include <QApplication>
#include <QMainWindow>

#include "maxon_epos2_driver/maxonInterface.h"

int main(int argc, char * argv[])
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    std::vector<std::string> non_ros_arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto rosNode = std::make_shared<rclcpp::Node>("maxon_epos2_driver");

    // parse options
    cmnCommandLineOptions options;
    std::string portName;
    double rosPeriod = 10.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;

    options.AddOptionOneValue("p", "port",
                              "Port name for the EPOS2 motor",
                              cmnCommandLineOptions::REQUIRED_OPTION, &portName);

    std::list<std::string> managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    if (!options.Parse(non_ros_arguments, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the Maxon motor interface
    maxonInterface * maxon_motor = new maxonInterface("MaxonMotor", portName);

    // add the component to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(maxon_motor);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // ROS CRTK bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("maxon_crtk_bridge", rosNode);
    crtk_bridge->bridge_interface_provided(maxon_motor->GetName(),
                                            "control",
                                            "",
                                            rosPeriod, tfPeriod);

    componentManager->AddComponent(crtk_bridge);
    crtk_bridge->Connect();
    
    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // Stop all logs
    cmnLogger::Kill();

    // stop ROS node
    rclcpp::shutdown();

    // Cleanup components
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
