/*!
 * Example main application for the maxon interface
 */

#include <QApplication>
#include <maxon_epos2_driver/maxonWidget.h>
#include <maxon_epos2_driver/maxonStatusWidget.h>
#include <maxon_epos2_driver/maxonControlWidget.h>

#include <maxon_epos2_driver/maxonInterface.h>
#include <maxon_epos2_driver/maxonMotor.h>
#include <cisstMultiTask/mtsManagerLocal.h>

#include <algorithm>

// this should really be something that include in a configuration file rather than a #if
// #define TEST_2_USB

int main(int argc, char **argv)
{
  cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
  cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
  cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
  cmnLogger::SetMaskClass("maxonMotor", CMN_LOG_ALLOW_ALL);
  cmnLogger::SetMaskClass("mtsManagerLocal", CMN_LOG_ALLOW_ALL);
  cmnLogger::SetMaskClass("maxonInterface", CMN_LOG_ALLOW_ALL);
  cmnLogger::SetMaskClass("maxonControlWidget", CMN_LOG_ALLOW_ALL);
  cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ALL);

  QApplication app(argc, argv);
  maxonInterface controller("maxonInterface", "USB0");
#ifdef TEST_2_USB
  maxonInterface controller2("maxonInterface2", "USB1");
#endif
  // bool doCollection = false;

  if (argc < 2)
  {
    std::cerr << "Usage: maxonUIControl config_file [do_collection]" << std::endl;
    std::cerr << "  config_file : The JSON motor configuration file" << std::endl;
    // std::cerr << "  do_collection : Optional argument to do file collection." << std::endl;
    // std::cerr << "                  Use the string true or 1 to do collection. Default is false." << std::endl;
    return 0;
  }

  // if (argc >= 3)
  //{
  //   std::string str = std::string(argv[2]);
  //   std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  //   if ((str.compare("true") == 0) || (str.compare("0") == 1))
  //     doCollection = true;
  // }

  // initialize the controller
  controller.initialize();

  // add the motors to the controller
  controller.addNodes(std::string(argv[1]));

  // construct the maxon widget
  maxonWidget widget(&controller);

#ifdef TEST_2_USB
  controller2.initialize();
  controller2.addNodes(std::string(argv[2]));
  maxonWidget m2(&controller2);
#endif

  // start the components
  mtsManagerLocal *componentManager = mtsManagerLocal::GetInstance();

  componentManager->AddComponent(&controller);
  int numMotors = controller.getNumMotors();
  for (int i = 0; i < numMotors; i++)
  {
    maxonMotor *m = controller.getMotor(i);
    std::cout << "adding motor " << m->GetName() << " to manager" << std::endl;
    componentManager->AddComponent(m);
    componentManager->AddComponent(widget.controls[i]);
  }

  componentManager->AddComponent(widget.statusWidget);

  /*
  #ifdef TEST_2_USB
    controller2.Create();
  #endif
    //if (doCollection)
    //  controller.createCollectors();

    componentManager->WaitForStateAll(mtsComponentState::READY, 2.0*cmn_s);
    controller.Start();
  #ifdef TEST_2_USB
    controller2.Start();
  #endif
    //if (doCollection)
    //  controller.startCollection();

    m.show();
  #ifdef TEST_2_USB
    m2.show();
  #endif
    app.exec();

    //if (doCollection)
    //  controller.stopCollection();
  */

  controller.Connect(componentManager);
  widget.Connect(componentManager);

  componentManager->CreateAll();
  componentManager->StartAll();
  widget.show();
  app.exec();
  componentManager->KillAll();
  componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);
  componentManager->Cleanup();

  widget.~maxonWidget();

  return 0;
}
