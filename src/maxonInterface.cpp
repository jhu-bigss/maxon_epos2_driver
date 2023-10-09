#include "maxon_epos2_driver/maxonInterface.h"
#include "maxon_epos2_driver/maxonMotor.h"
#include "maxon_epos2_driver/maxonMotorInterface.h"

#include <cstring>
#include <cmath>
#include <fstream>

#include "Definitions.h"

#include <json/json.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsCollectorState.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstOSAbstraction/osaSleep.h>

#ifndef WIN32
typedef int BOOL;
#endif

CMN_IMPLEMENT_SERVICES_DERIVED(maxonInterface, mtsComponent)

////////////////////
// maxonInterface //
////////////////////
maxonInterface::maxonInterface(const std::string &devName, const std::string &portName) :
  mtsComponent(devName),
  controller(0),
  errorCode(0),
  protocolStack("MAXON SERIAL V2"),
  ifaceName("USB"),
  devName("EPOS2"),
  portName(portName),
  baudrate(1000000),
  connected(false)
{
}

maxonInterface::~maxonInterface()
{
  close();

  for(size_t i=0; i<motorInterfaces.size(); i++)
    delete motorInterfaces[i];
}

void maxonInterface::Startup()
{
  // Start the collectors
  // for(int i=0; i<collectors.size(); i++) {
  //  collectors[i]->StartCollection(0.0);
  // }
}

void maxonInterface::Cleanup()
{
  // Stop the collectors
  // for(int i=0; i<collectors.size(); i++) {
  //  collectors[i]->StopCollection(0.0);
  // }
}

void maxonInterface::Connect(mtsManagerLocal *localManager)
{
  int numMotors = motorTasks.size();
  for (int i = 0; i < numMotors; i++)
  {
    std::string name = motorTasks[i]->GetName();
    localManager->Connect(this->GetName(), name+"ControlInterface", name, "control");
    localManager->Connect(this->GetName(), name+"StateInterface", name, "state");
  }
}

bool maxonInterface::addNodes(const std::string &filename)
{
  Json::Value root;
  Json::Reader reader;
  CMN_LOG_CLASS_RUN_VERBOSE << "loading nodes from file: " << filename.c_str() << std::endl;
  std::ifstream config(filename);
  bool ok = reader.parse(config, root, false);
  if (!ok) {
    std::cerr << "Error in parsing maxon configuration file." << std::endl;
    return false;
  }

#ifndef BUILD_MAXON_SIM
  //VCS_ResetDevice(controller, 1, &errorCode);
  VCS_SendNMTService(controller, 0, NCS_RESET_NODE, &errorCode);
  osaSleep(100 * cmn_ms); // add sleep here to let the reset actually do something

  VCS_SendNMTService(controller, 0, NCS_RESET_COMMUNICATION, &errorCode);
  VCS_SendNMTService(controller, 0, NCS_ENTER_PRE_OPERATIONAL, &errorCode);
  VCS_SendNMTService(controller, 0, NCS_START_REMOTE_NODE, &errorCode);
#endif

  std::string file, taskName, path;
  int nodeId;
  double cal, exc;

  // get the path such that the setup file can use relative paths
  size_t found = filename.find_last_of("/\\");
  path = filename.substr(0, found+1);

  for(Json::Value::iterator it = root["motors"].begin();
    it != root["motors"].end();
    ++it) {

      file = (*it)["file"].asString();
      nodeId = (*it)["nodeId"].asInt();
      taskName = (*it)["taskName"].asString();
      cal = (*it)["loadCell"]["cal"].asDouble();
      exc = (*it)["loadCell"]["excitation"].asDouble();
      addNode((short)nodeId, path + file, taskName, cal * exc);
  }

  if (motorInterfaces.size() >= 1)
    motorTasks[0]->clearFault();

  return true;
}

void maxonInterface::addNode(const unsigned short id,
    const std::string &filename, const std::string &taskName,
    const double &loadCellCal)
{
  // create the motor
  maxonMotor *mtr = new maxonMotor(id, filename, controller, taskName);
  mtr->setController( controller );
  motorTasks.push_back(mtr);


  // create the interface
  mtsInterfaceRequired *controlInterface = AddInterfaceRequired(taskName+"ControlInterface");
  mtsInterfaceRequired *stateInterface = AddInterfaceRequired(taskName+"StateInterface");
  maxonMotorInterface *m = new maxonMotorInterface(controlInterface, stateInterface);
  motorInterfaces.push_back(m);

  if (loadCellCal > 0) {
    mtr->setForceGain(loadCellCal);
    mtr->setUseAI(true);
  }
}

std::vector<std::string> maxonInterface::getMotorList()
{
  std::vector<std::string> list;
  int numMotors = motorTasks.size();
  for (int i = 0; i < numMotors; i++) list.push_back(motorTasks[i]->GetName());
  return list;
}

void maxonInterface::connectMotor(maxonMotor *m)
{
  mtsManagerLocal *componentManager = mtsManagerLocal::GetInstance();
  componentManager->AddComponent(m);

  componentManager->Connect(this->GetName(), m->GetName() + "Control",
    m->GetName(), "control");
  componentManager->Connect(this->GetName(), m->GetName() + "State",
    m->GetName(), "state");

  m->Create();
  componentManager->WaitForStateAll(mtsComponentState::READY, 500*cmn_ms);
  m->Start();
}

void maxonInterface::createCollectors()
{
  for(size_t i=0; i<motorInterfaces.size(); i++) {
    mtsCollectorState *collector = new mtsCollectorState(motorTasks[i]->GetName(),
      motorTasks[i]->GetDefaultStateTableName(),
      mtsCollectorBase::COLLECTOR_FILE_FORMAT_CSV);

    // add data to the collector
    collector->AddSignal("position");
    collector->AddSignal("velocity");
    collector->AddSignal("time");
    collector->AddSignal("current");

    if (motorTasks[i]->getUseAI())
      collector->AddSignal("force");

    collector->AddSignal("numCommandsProcessed");
    collector->AddSignal("startTime");
    collector->AddSignal("endTime");
    collector->AddSignal("setVelocityStartTime");
    collector->AddSignal("setVelocityEndTime");
    collector->AddSignal("moveVelocityStartTime");
    collector->AddSignal("moveVelocityEndTime");
    collector->AddSignal("stopStartTime");
    collector->AddSignal("stopEndTime");

    collectors.push_back(collector);
  }
}

void maxonInterface::addCollectors(mtsManagerLocal *componentManager)
{
  for (size_t i = 0; i < collectors.size(); i++)
  {
    componentManager->AddComponent(collectors[i]);
    collectors[i]->Connect();
    collectors[i]->Create();
    collectors[i]->Start();
  }
}

maxonMotor *maxonInterface::getMotor(const unsigned int id)
{
  if(id >= motorTasks.size())
    return 0;

  return motorTasks[id];
}

maxonMotorInterface *maxonInterface::getMotorInterface(const unsigned int id)
{
  if (id >= motorInterfaces.size())
    return 0;

  return motorInterfaces[id];
}

bool maxonInterface::initialize()
{
  char name[255];
  char stack[255];
  char iface[255];
  char port[255];

  strcpy(name, devName.c_str());
  strcpy(stack, protocolStack.c_str());
  strcpy(iface, ifaceName.c_str());
  strcpy(port, portName.c_str());
  
  DWORD lBaudrate, timeout;
  lBaudrate = timeout = 0;

#ifndef BUILD_MAXON_SIM
  // use OpenDevice since this is available on both systems
  controller = VCS_OpenDevice(name, stack, iface, port, &errorCode);
  CMN_LOG_CLASS_INIT_DEBUG << "found controller handle " << controller << std::endl;
  if(controller == 0 && errorCode != 0) {
    controller = 0;
    connected = false;
    return connected;
  }


  // I think Windows does this internally in VCS_OpenDeviceDlg...
  if(VCS_GetProtocolStackSettings(controller, &lBaudrate, &timeout, &errorCode) != 0) {
    if(VCS_SetProtocolStackSettings(controller, baudrate, timeout, &errorCode) != 0) {
      if(VCS_GetProtocolStackSettings(controller, &lBaudrate, &timeout, &errorCode) != 0) {
        if(baudrate == (int)lBaudrate) {
          connected = true;
          return connected;
        }
      }
    }
  }
#endif
  return false;
}

void maxonInterface::printCommunicationSelections()
{
#ifndef BUILD_MAXON_SIM
  char name[255];
  char stack[255];
  char iface[255];
  char port[255];

  strcpy(name, devName.c_str());
  strcpy(stack, protocolStack.c_str());
  strcpy(iface, ifaceName.c_str());
  strcpy(port, portName.c_str());
  
  char buffer[1000];
  int end = 0;
  int start = 1;
  std::cout <<  "PROTOCOLS" << std::endl;

  while(!end) {
    VCS_GetProtocolStackNameSelection(name, start, buffer, 1000, &end, &errorCode);
    std::cout << buffer << std::endl;
    start = 0;
  }

  end = 0;
  start = 1;
  std::cout << "INTERFACES" <<std::endl;

  while(!end) {
    VCS_GetInterfaceNameSelection(name, stack, start, buffer, 1000, &end, &errorCode);
    std::cout << buffer << std::endl;
    start = 0;
  }

  end = 0;
  start = 1;
  std::cout << "PORTS" << std::endl;

  while(!end) {
    VCS_GetPortNameSelection(name, stack, iface, start, buffer, 1000, &end, &errorCode);
    std::cout << buffer << std::endl;
    start = 0;
  }
#endif
}

bool maxonInterface::close()
{
  disable();
  if(!connected) {
    return true;
  }

  connected = false;
#ifndef BUILD_MAXON_SIM
  if(VCS_CloseDevice(controller, &errorCode) != 0 && errorCode == 0)
    return true;
#endif

  return false;
}

void maxonInterface::disable()
{
  for(size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->disable();

}

void maxonInterface::setVelocity(const double vel)
{
  for(size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->setVelocity(vel);
}

void maxonInterface::setVelocity(const unsigned int &motorId, const double vel)
{
  if (motorId < motorInterfaces.size())
  {
    motorInterfaces[motorId]->setVelocity(vel);
  }
}

void maxonInterface::setVelocity_raw(const long vel)
{
  for(size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->setVelocity_raw(vel);
}

void maxonInterface::setVelocity_raw(const unsigned int &motorId, const long &vel)
{
  if (motorId < motorInterfaces.size())
  {
    motorInterfaces[motorId]->setVelocity_raw(vel);
  }
}

void maxonInterface::moveToPosition(const double *pos, const bool abs)
{
  for(size_t i=0; i<motorInterfaces.size(); i++) {
    motorInterfaces[i]->setPositionMode();
    if(abs)
      motorInterfaces[i]->moveToAbsolutePosition(pos[i]);
    else
      motorInterfaces[i]->moveToRelativePosition(pos[i]);
  }
}

void maxonInterface::moveToPosition(std::vector<double> pos, const bool abs)
{
  for(size_t i=0; i<motorInterfaces.size(); i++) {
    motorInterfaces[i]->setPositionMode();
    if(abs)
      motorInterfaces[i]->moveToAbsolutePosition(pos[i]);
    else
      motorInterfaces[i]->moveToRelativePosition(pos[i]);
  }
}

void maxonInterface::moveToPosition(const vctDoubleVec &pos, const bool abs)
{
  if(pos.size() > motorInterfaces.size()) {
    std::cout << "bad size: " << pos.size() << " " << motorInterfaces.size() << std::endl;
    return;
  }

  moveToPosition(pos.Pointer(), abs);
}

void maxonInterface::moveToPosition(const unsigned int &motorId, const double &pos, const bool abs)
{
  if (motorId < motorInterfaces.size())
  {
    if (abs)
      motorInterfaces[motorId]->moveToAbsolutePosition(pos);
    else
      motorInterfaces[motorId]->moveToRelativePosition(pos);
  }
}

void maxonInterface::getPositions(std::vector<double> &pos)
{
  for(size_t i=0; i<motorInterfaces.size(); i++) {
    double p;
    motorInterfaces[i]->position(p);
    pos.push_back(p);
  }
}

void maxonInterface::getPositions(vctDoubleVec &pos)
{
  pos.resize(motorInterfaces.size());
  for(size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->position(pos[i]);
}

void maxonInterface::getPositiveLimits(vctDoubleVec &limits)
{
  limits.resize(motorInterfaces.size());
  for (size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->positiveLimit(limits[i]);
}

void maxonInterface::getNegativeLimits(vctDoubleVec &limits)
{
  limits.resize(motorInterfaces.size());
  for (size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->negativeLimit(limits[i]);
}

void maxonInterface::getForce(vctDoubleVec &force)
{
  force.resize(motorInterfaces.size());
  for (size_t i=0; i<motorInterfaces.size(); i++)
    motorInterfaces[i]->force(force[i]);
}

bool maxonInterface::targetReached()
{
  bool allReached = true;

#ifndef BUILD_MAXON_SIM
#warning "building maxon without sim"
  bool reached = false;
  //long target = 0;
  BOOL rr;
  for(size_t i=0; i<motorInterfaces.size(); i++) {
    unsigned short nodeId;
    motorInterfaces[i]->getNodeId(nodeId);
    VCS_GetMovementState(this->controller, nodeId, &rr, &errorCode);
    reached = (rr != 0);

    // calling the MTS function below does not seem to return the corrent result
    //motorInterfaces[i]->targetReached(reached);
    allReached &= reached;
  }
#endif

  return allReached;
}

std::string maxonInterface::getErrorString()
{
  char error[100];
  std::string errorStr = "No error";

#ifndef BUILD_MAXON_SIM
  if(VCS_GetErrorInfo(errorCode, error, 100))
    errorStr = std::string(error);
#endif

  return errorStr;
}

void maxonInterface::startCollection()
{
  for(size_t i=0; i<collectors.size(); i++) {
    collectors[i]->StartCollection(0.0);
  }
}

void maxonInterface::stopCollection()
{
  for(size_t i=0; i<collectors.size(); i++) {
    collectors[i]->StopCollection(0.0);
  }
}

void maxonInterface::setDigitalOutput(
  const std::vector<int> &vals) {
  for (size_t i = 0; i < motorInterfaces.size(); i++) {
    motorInterfaces[i]->setDigitalOutput(vals[i]);
  }
}

void maxonInterface::setVelocityMode()
{
  for (size_t i = 0; i < motorInterfaces.size(); i++) {
    motorInterfaces[i]->setVelocityMode();
  }
}

