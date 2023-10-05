#include "maxon_epos2_driver/maxonMotor.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <climits>

#include "Definitions.h"

#include <json/json.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstParameterTypes/prmRobotState.h>

#ifndef WIN32
typedef int BOOL;
#endif

CMN_IMPLEMENT_SERVICES_DERIVED(maxonMotor, mtsTaskPeriodic)

maxonMotor::maxonMotor(const unsigned short id, HANDLE c, const std::string &taskName) :
  mtsTaskPeriodic(taskName, 100.0*cmn_ms),
  controller(c),
  errorCode(0),
  nodeId(id),
  accel(10000),
  decel(10000),
  zeroPos(0),  
  targetVelocity(0),
  pitch(1.0),
  gearRatio(1),
  quadrature(4),
  countsPerTurn(512),
  negLimit(-100),
  posLimit(100),
  stroke(20.0),
  enabled(false),
  fault(false),
  targetReached(false),
  hasAI(false),
  aiCal(5.0/4096.0), // based on maxon spec sheet
  forceGain(1.0),
  homing(false),
  homingDesForce(0.2),
  homingThresh(0.02),
  analogInput_bias(0.0),
  operationalMode(OMD_PROFILE_POSITION_MODE),
  numCommandsProcessed(0),
  startTime(-1.0),
  endTime(-1.0),
  setVelocityStartTime(-1.0),
  setVelocityEndTime(-1.0),
  moveVelocityStartTime(-1.0),
  moveVelocityEndTime(-1.0),
  stopStartTime(-1.0),
  stopEndTime(-1.0),
  velocitySetPoint(0.0),
  setPointMode(false),
  lastVelocityCommand(bigss::time_now_ms()),
  lastVelocityCommandThresh(1000)
{
  createInterface();

  clearFault();
  disable();
}

maxonMotor::maxonMotor(const unsigned short id, const std::string &filename, HANDLE c, const std::string &taskName) :
  mtsTaskPeriodic(taskName, 100.0*cmn_ms),
  controller(c),
  errorCode(0),
  nodeId(id),
  accel(10000),
  decel(10000),
  zeroPos(0),  
  targetVelocity(0),
  pitch(1.0),
  gearRatio(1),
  quadrature(4),
  countsPerTurn(512),
  negLimit(-100),
  posLimit(100),
  stroke(20.0),
  enabled(false),
  fault(false),
  targetReached(false),
  hasAI(false),
  aiCal(5.0/4096.0), // based on maxon spec sheet
  forceGain(1.0),
  homing(false),
  homingDesForce(0.2),
  homingThresh(0.02),
  analogInput_bias(0.0),
  operationalMode(OMD_PROFILE_POSITION_MODE),
  numCommandsProcessed(0),
  startTime(-1.0),
  endTime(-1.0),
  setVelocityStartTime(-1.0),
  setVelocityEndTime(-1.0),
  moveVelocityStartTime(-1.0),
  moveVelocityEndTime(-1.0),
  stopStartTime(-1.0),
  stopEndTime(-1.0),
  velocitySetPoint(0.0),
  setPointMode(false),
  lastVelocityCommand(bigss::time_now_ms()),
  lastVelocityCommandThresh(1000)
{
  createInterface();

  clearFault();
  disable();

  loadMotorConfiguration(filename);
}

maxonMotor::~maxonMotor()
{

}

void maxonMotor::Startup()
{

}

void maxonMotor::Cleanup()
{
  disable();
}

void maxonMotor::Configure(const std::string &filename)
{
  loadMotorConfiguration(filename);
}

void maxonMotor::resetTimingVariables()
{
  setVelocityStartTime = -1.0;
  setVelocityEndTime = -1.0;
  moveVelocityStartTime = -1.0;
  moveVelocityEndTime = -1.0;
  stopStartTime = -1.0;
  stopEndTime = -1.0;
}

void maxonMotor::Run()
{
  startTime = bigss::time_now_ms();
  resetTimingVariables();
  numCommandsProcessed = ProcessQueuedCommands();

  // this is a hack to read the system time
  std::string t;
  osaGetTimeString(t, osaGetTime(), ':');
  time = t;

  // update the statu
  if(enabled)
    updateStatus();

  if(homing)
    checkForceHoming();

  if (setPointMode)
  {
    CMN_LOG_CLASS_RUN_DEBUG << "MOVING " << this->Name << " IN SET POINT MODE " << velocitySetPoint << std::endl;
    bool nonZeroSetPoint = (std::abs(velocitySetPoint) > 0.0001);
    bool timedOut = bigss::time_now_ms() - lastVelocityCommand > lastVelocityCommandThresh;

    // if there's a nonzero command, and we haven't timed out OR there's no timeout
    // in place, then command the motor
    if (nonZeroSetPoint && (!timedOut || lastVelocityCommandThresh == 0))
    {
      VCS_MoveWithVelocity(controller, nodeId, velocitySetPoint, &errorCode);
    }
    else
    {
      VCS_HaltVelocityMovement(controller, nodeId, &errorCode);
    }

  }

  endTime = bigss::time_now_ms();
}

void maxonMotor::createInterface()
{
  // Construct the state table
  StateTable.AddData(operationMode, "operationMode");
  StateTable.AddData(enabled, "enabled");
  StateTable.AddData(position_raw, "position_raw");
  StateTable.AddData(velocity_raw, "velocity_raw");
  StateTable.AddData(velocityAverage_raw, "velocityAverage_raw");
  StateTable.AddData(position, "position");
  StateTable.AddData(velocity, "velocity");
  StateTable.AddData(velocityAverage, "velocityAverage");
  StateTable.AddData(current, "current");
  StateTable.AddData(currentAverage, "currentAverage");
  StateTable.AddData(fault, "fault");
  StateTable.AddData(time, "time");
  StateTable.AddData(targetReached, "targetReached");
  StateTable.AddData(force, "force");

  StateTable.AddData(numCommandsProcessed, "numCommandsProcessed");
  StateTable.AddData(startTime, "startTime");
  StateTable.AddData(endTime, "endTime");
  StateTable.AddData(setVelocityStartTime, "setVelocityStartTime");
  StateTable.AddData(setVelocityEndTime, "setVelocityEndTime");
  StateTable.AddData(moveVelocityStartTime, "moveVelocityStartTime");
  StateTable.AddData(moveVelocityEndTime, "moveVelocityEndTime");
  StateTable.AddData(stopStartTime, "stopStartTime");
  StateTable.AddData(stopEndTime, "stopEndTime");
  
  mtsInterfaceProvided *provided = AddInterfaceProvided("state");
  if(provided) {
    provided->AddCommandReadState(StateTable, operationMode, "operationMode");
    provided->AddCommandReadState(StateTable, enabled, "enabled");
    provided->AddCommandReadState(StateTable, position_raw, "position_raw");
    provided->AddCommandReadState(StateTable, velocity_raw, "velocity_raw");
    provided->AddCommandReadState(StateTable, velocityAverage_raw, "velocityAverage_raw");
    provided->AddCommandReadState(StateTable, position, "position");
    provided->AddCommandReadState(StateTable, velocity, "velocity");
    provided->AddCommandReadState(StateTable, velocityAverage, "velocityAverage");
    provided->AddCommandReadState(StateTable, current, "current");
    provided->AddCommandReadState(StateTable, currentAverage, "currentAverage");
    provided->AddCommandReadState(StateTable, fault, "fault");
    provided->AddCommandReadState(StateTable, time, "time");
    provided->AddCommandReadState(StateTable, force, "force");
    //provided->AddCommandReadState(StateTable, targetReached, "targetReached");
  } else
    std::cerr << "[maxonMotor] Could not add state interface" << std::endl;

  provided = AddInterfaceProvided("control");
  if(provided) {
    provided->AddCommandVoidReturn(&maxonMotor::enable, this, "enable", mtsBool());
    provided->AddCommandVoid(&maxonMotor::disable, this, "disable");
    provided->AddCommandVoid(&maxonMotor::clearFault, this, "clearFault");
    provided->AddCommandVoid(&maxonMotor::stop, this, "stop");
    provided->AddCommandWrite(&maxonMotor::setVelocity, this, "setVelocity", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setVelocity_raw, this, "setVelocity_raw", mtsLong());
    provided->AddCommandWrite(&maxonMotor::setVelocityPoint, this, "setVelocityPoint", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setVelocityPoint_raw, this, "setVelocityPoint_raw", mtsDouble());
    provided->AddCommandVoid(&maxonMotor::jogPlus, this, "jogPlus");
    provided->AddCommandVoid(&maxonMotor::jogMinus, this, "jogMinus");
    provided->AddCommandWrite(&maxonMotor::moveToAbsolutePosition, this, "moveToAbsolutePosition", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::moveToRelativePosition, this, "moveToRelativePosition", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::moveToAbsolutePosition_raw, this, "moveToAbsolutePosition_raw", mtsLong());
    provided->AddCommandWrite(&maxonMotor::moveToRelativePosition_raw, this, "moveToRelativePosition_raw", mtsLong());
    provided->AddCommandRead(&maxonMotor::getPositiveLimit, this, "positiveLimit", mtsDouble());
    provided->AddCommandRead(&maxonMotor::getNegativeLimit, this, "negativeLimit", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setPositiveLimit, this, "setPositiveLimit", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setNegativeLimit, this, "setNegativeLimit", mtsDouble());
    provided->AddCommandVoid(&maxonMotor::setNegativeLimitCurrentPosition, this, "setNegativeLimitCurrentPosition");
    provided->AddCommandVoid(&maxonMotor::setPositiveLimitCurrentPosition, this, "setPositiveLimitCurrentPosition");
    provided->AddCommandVoidReturn(&maxonMotor::getTargetVelocity, this, "getTargetVelocity", mtsDouble());
    provided->AddCommandVoidReturn(&maxonMotor::getTargetVelocity_raw, this, "getTargetVelocity_raw", mtsLong());
    provided->AddCommandVoid(&maxonMotor::setPositionMode, this, "setPositionMode");
    provided->AddCommandVoid(&maxonMotor::setVelocityMode, this, "setVelocityMode");
    provided->AddCommandWrite(&maxonMotor::setSetPointMode, this, "setSetPointMode", mtsBool());
    provided->AddCommandVoidReturn(&maxonMotor::getErrorString, this, "getErrorString", mtsStdString());
    provided->AddCommandRead(&maxonMotor::getNodeId, this, "getNodeId", mtsUShort());
    provided->AddCommandVoid(&maxonMotor::zero, this, "zero");
    provided->AddCommandVoid(&maxonMotor::moveToZero, this, "moveToZero");
    provided->AddCommandWrite(&maxonMotor::setDigitalOutput, this, "setDigitalOutput");
    provided->AddCommandVoid(&maxonMotor::rebias, this, "rebias");
    provided->AddCommandWrite(&maxonMotor::setHomeForce, this, "setHomeForce", mtsDouble());
    provided->AddCommandVoid(&maxonMotor::startForceHoming, this, "startForceHoming");
  } else
    std::cerr << "[maxonMotor] Could not add control interface" << std::endl;
}

void maxonMotor::getNodeId(mtsUShort &id) const 
{
  id = nodeId;
}

void maxonMotor::clearFault()
{
  BOOL fault;
  VCS_SendNMTService(controller, nodeId, NCS_RESET_NODE, &errorCode);
  osaSleep(100 * cmn_ms); // add sleep here to let the reset actually do something

  VCS_SendNMTService(controller, nodeId, NCS_RESET_COMMUNICATION, &errorCode);
  VCS_GetFaultState(controller, nodeId, &fault, &errorCode);
  VCS_ClearFault(controller, nodeId, &errorCode);
}

void maxonMotor::zero()
{
  // should this grab the information from the state table? 
  double currPos = position;
  zeroPos += position_raw;
  posLimit += currPos;
  negLimit += currPos;
}

void maxonMotor::moveToZero()
{
  moveToPosition_raw(0, true);
}

bool maxonMotor::loadMotorConfiguration(const std::string &filename)
{
  Json::Value root;
  Json::Reader reader;
  std::ifstream config(filename.data());
  bool ok = reader.parse(config, root, false);
  if (!ok) {
    return false;
  }

  BOOL fault;
  if(!VCS_GetFaultState(controller, nodeId, &fault, &errorCode)) {
    mtsStdString s;
    this->getErrorString(s);
    CMN_LOG_CLASS_INIT_ERROR << "could not get fault state " << s.Data << std::endl;
  }

  //if(fault) {
    if(!VCS_ClearFault(controller, nodeId, &errorCode)) {
      mtsStdString s;
      this->getErrorString(s);
      CMN_LOG_CLASS_INIT_ERROR << "fault " << s.Data << std::endl;
    }
  //}

  if(VCS_SetMotorType(controller, nodeId, MT_DC_MOTOR, &errorCode) == 0) {
    CMN_LOG_CLASS_INIT_ERROR << "Could not set motor type for node " << nodeId << std::endl;
    mtsStdString s;
    this->getErrorString(s);
    std::cerr << s.Data << std::endl;
  }

  // stroke
  stroke = root["motor"]["motorParams"].get("stroke", 20).asDouble();

  // motor parameters
  WORD nominalCurrent = static_cast<WORD>(root["motor"]["motorParams"].get("nominalCurrent", 569).asInt());
  WORD maxCurrent = static_cast<WORD>(root["motor"]["motorParams"].get("maxCurrent", 1138).asInt());
  WORD thermalTimeConstant = static_cast<WORD>(root["motor"]["motorParams"].get("thermalTimeConstant", 106).asInt());
  pitch = root["motor"]["motorParams"].get("pitch", 1).asDouble();
  gearRatio = root["motor"]["motorParams"].get("gearRatio", 16).asDouble();
  if(VCS_SetDcMotorParameter(controller, nodeId, nominalCurrent, maxCurrent, thermalTimeConstant, &errorCode) == 0) {
    CMN_LOG_CLASS_INIT_ERROR << "Could not set dc motor parameters for node " << nodeId << std::endl;
    mtsStdString s;
    this->getErrorString(s);
    std::cerr << s.Data << std::endl;
  }

  // sensor parameters
  DWORD encoderResolution = static_cast<DWORD>(root["motor"]["sensorParams"].get("encoderResolution", 512).asInt());
  BOOL invertedPolarity = static_cast<BOOL>(root["motor"]["sensorParams"].get("invertedPolarity", 0).asInt());
  quadrature = root["motor"]["sensorParams"].get("quadrature", 4).asInt();
  WORD sensorType = static_cast<WORD>(root["motor"]["sensorParams"].get("type", 2).asInt());
  VCS_SetSensorType(controller, nodeId, sensorType, &errorCode);
  VCS_SetIncEncoderParameter(controller, nodeId, encoderResolution, invertedPolarity, &errorCode);
  countsPerTurn = (int)encoderResolution;
  
  // safety parameters
  DWORD maxFollowError = static_cast<DWORD>(root["motor"]["safetyParams"].get("maxFollowError", 2000).asLargestUInt());
  DWORD maxProfileVelocity = static_cast<DWORD>(root["motor"]["safetyParams"].get("maxProfileVelocity", 14000).asLargestUInt());
  DWORD maxAccel = static_cast<DWORD>(root["motor"]["safetyParams"].get("maxAccel", 0xffffffff).asLargestUInt());
  VCS_SetMaxFollowingError(controller, nodeId, maxFollowError, &errorCode);
  VCS_SetMaxProfileVelocity(controller, nodeId, maxProfileVelocity, &errorCode);
  VCS_SetMaxAcceleration(controller, nodeId, maxAccel, &errorCode);

  // position regulator
  WORD p = static_cast<WORD>(root["motor"]["positionRegulator"].get("p", 65).asInt());
  WORD i = static_cast<WORD>(root["motor"]["positionRegulator"].get("i", 345).asInt());
  WORD d = static_cast<WORD>(root["motor"]["positionRegulator"].get("d", 65).asInt());
  WORD vFF= static_cast<WORD>(root["motor"]["positionRegulator"].get("vFF", 0).asInt());
  WORD aFF = static_cast<WORD>(root["motor"]["positionRegulator"].get("aFF", 14).asInt());
  VCS_SetPositionRegulatorGain(controller, nodeId, p, i, d, &errorCode);
  VCS_SetPositionRegulatorFeedForward(controller, nodeId, vFF, aFF, &errorCode);

  // velocity regulator
  p = static_cast<WORD>(root["motor"]["velocityRegulator"].get("p", 173).asInt());
  i = static_cast<WORD>(root["motor"]["velocityRegulator"].get("i", 43).asInt());
  vFF= static_cast<WORD>(root["motor"]["velocityRegulator"].get("vFF", 0).asInt());
  aFF = static_cast<WORD>(root["motor"]["velocityRegulator"].get("aFF", 14).asInt());
  VCS_SetVelocityRegulatorGain(controller, nodeId, p, i, &errorCode);
  VCS_SetVelocityRegulatorFeedForward(controller, nodeId, vFF, aFF, &errorCode);

  // current regulator
  p = static_cast<WORD>(root["motor"]["currentRegulator"].get("p", 732).asInt());
  i = static_cast<WORD>(root["motor"]["currentRegulator"].get("i", 451).asInt());
  VCS_SetCurrentRegulatorGain(controller, nodeId, p, i, &errorCode);

  return true;
}


void maxonMotor::updateStatus()
{
  char opMode;
  WORD ai1;

  int fault = 0;
  if(!VCS_GetFaultState(controller, nodeId, &fault, &errorCode)) {
    fault = false;
  }
  this->fault.Data = (fault != 0);

  if(VCS_GetOperationMode(controller, nodeId, &opMode, &errorCode)) {
    switch(opMode) {
      case OMD_PROFILE_POSITION_MODE:
        operationMode.Data = "Profile position";
        break;
      case OMD_PROFILE_VELOCITY_MODE:
        operationMode.Data = "Profile velocity";
        break;
      case OMD_HOMING_MODE:
        operationMode.Data = "Homing";
        break;
      case OMD_INTERPOLATED_POSITION_MODE:
        operationMode.Data = "Interpolated position";
        break;
      case OMD_POSITION_MODE:
        operationMode.Data = "Position";
        break;
      case OMD_VELOCITY_MODE:
        operationMode.Data = "Velocity";
        break;
      case OMD_CURRENT_MODE:
        operationMode.Data = "Current";
        break;
      case OMD_MASTER_ENCODER_MODE:
        operationMode.Data = "Master encoder";
        break;
      case OMD_STEP_DIRECTION_MODE:
        operationMode.Data = "Step direction";
        break;
      default:
        operationMode.Data = "Unknown";
    }
  }

  int enabled_;
  if(!VCS_GetEnableState(controller, nodeId, &enabled_, &errorCode))
    enabled_ = 0;

  enabled.Data = (enabled_ != 0) && !this->fault.Data;

  //long pos = 0;
  VCS_GetPositionIs(controller, nodeId, &position_raw.Data, &errorCode);
  position_raw.Data -= zeroPos; // zero out the position
  VCS_GetVelocityIs(controller, nodeId, &velocity_raw.Data, &errorCode);
  VCS_GetVelocityIsAveraged(controller, nodeId, &velocityAverage_raw.Data, &errorCode);
  VCS_GetCurrentIs(controller, nodeId, &current.Data, &errorCode);
  VCS_GetCurrentIsAveraged(controller, nodeId, &currentAverage.Data, &errorCode);

  // convert measures to appropriate units
  position = encoderCountsToUnits(position_raw);
  velocity = rpmToUnits(velocity_raw);
  velocityAverage = rpmToUnits(velocityAverage_raw);

  // analog input
  if (hasAI) {
    VCS_GetAnalogInput(controller, nodeId, 1, &ai1, &errorCode);
   // VCS_GetAnalogInput(controller, nodeId, 2, &ai2, &errorCode);
   // analogInput_raw = aiToVolts(ai1) - aiToVolts(ai2);
	  analogInput_raw = aiToVolts(ai1);
    force = voltsToForce(analogInput_raw - analogInput_bias);
  } else {
    analogInput_raw = 0.0;
    force = 0.0;
  }

  // update if the target has been reached
  BOOL reached;
  VCS_GetMovementState(controller, nodeId, &reached, &errorCode);
  targetReached = (reached != 0);
}

bool maxonMotor::getStatus(struct status &stat)
{
  stat.position = position;
  stat.position_raw = position_raw;
  stat.velocity = velocity;
  stat.velocity_raw = velocity_raw;
  stat.velocityAverage = velocityAverage;
  stat.velocityAverage_raw = velocityAverage_raw;
  stat.operationMode = operationMode;
  stat.current = current;
  stat.currentAverage = current;
  stat.force = force;
  stat.ai_raw = analogInput_raw;

  return enabled;
}

void maxonMotor::setHomeForce(const mtsDouble &homeForce)
{
  homingDesForce = homeForce.Data;
}

void maxonMotor::startForceHoming(){
  double forceDiff = homingDesForce - double(force);
  mtsBool result;
  if (forceDiff > 0) {
    jogPlus();
  } else {
    jogMinus();
  }

  homing = true;
}

void maxonMotor::checkForceHoming()
{
  double forceDiff = std::abs(homingDesForce - double(force));
  if (forceDiff < homingThresh){
    stop();
    zero(); // set cuurent position as zero position
    homing = false;
  }
}

void maxonMotor::rebias()
{
  WORD ai1;
  // analog input
  if (hasAI) {
    VCS_GetAnalogInput(controller, nodeId, 1, &ai1, &errorCode);
    analogInput_bias = aiToVolts(ai1); 
  }
}
void maxonMotor::enable(mtsBool &result)
{
  result = false;
  int fault = 0;
  CMN_LOG_CLASS_INIT_DEBUG << "enabling controller " << controller << " with node id " << nodeId << std::endl;
  if(!VCS_GetFaultState(controller, nodeId, &fault, &errorCode)) {
    return;
  }

  if(fault) {
    if(!VCS_ClearFault(controller, nodeId, &errorCode)) {
      return;
    }
  }

  if(!VCS_SetEnableState(controller, nodeId, &errorCode)) {
    mtsStdString s;
    this->getErrorString(s);
    return;
  }

  // set the accel/decel profile limits
  setVelocityProfile();
  setPositionProfile();
  setPositionMode();

  if(VCS_GetOperationMode(controller, nodeId, &operationalMode, &errorCode)) {
    switch(operationalMode) {
      case OMD_PROFILE_POSITION_MODE:
	      getPositionProfile();
        break;
      case OMD_PROFILE_VELOCITY_MODE:
        getVelocityProfile();
        break;
      case OMD_HOMING_MODE:
      case OMD_INTERPOLATED_POSITION_MODE:
      case OMD_POSITION_MODE:
      case OMD_VELOCITY_MODE:
      case OMD_CURRENT_MODE:
      case OMD_MASTER_ENCODER_MODE:
      case OMD_STEP_DIRECTION_MODE:
      default:
	      getPositionProfile();
    }
  }

  enabled = true;
  result = true;
}

void maxonMotor::setDigitalOutput(const mtsInt &val) {

  // variables
  //bool success = false;

  if (val != 0) {
    CMN_LOG_CLASS_RUN_DEBUG << "Setting " << this->GetName() << " digital output to high" << std::endl;
    /*success = */VCS_DigitalOutputConfiguration(controller, nodeId, 3, 15, 0,
      1, 1, &errorCode);
  } else {
    CMN_LOG_CLASS_RUN_DEBUG << "Setting " << this->GetName() << " digital output to low" << std::endl;
    /*success = */VCS_DigitalOutputConfiguration(controller, nodeId, 3, 15, 1,
      1, 1, &errorCode);
  }

  // set digital outputs
  // bool success = VCS_SetAllDigitalOutputs(controller, nodeId, output, &errorCode);
}

void maxonMotor::disable()
{
  if(!enabled)
    return;

  stop();
  enabled = false;
  if(!VCS_SetDisableState(controller, nodeId, &errorCode)) {
    mtsStdString s;
    this->getErrorString(s);
    CMN_LOG_CLASS_INIT_ERROR << "Error: " << s.GetData() << std::endl;
  }
}

void maxonMotor::stop()
{
  stopStartTime = bigss::time_now_ms();

  switch(operationalMode) {
    case OMD_PROFILE_POSITION_MODE:
      VCS_HaltPositionMovement(controller, nodeId, &errorCode);
      break;
    case OMD_PROFILE_VELOCITY_MODE:
      VCS_HaltVelocityMovement(controller, nodeId, &errorCode);
      break;
    // None of these other modes should be used
    case OMD_HOMING_MODE:
      VCS_StopHoming(controller, nodeId, &errorCode);
      break;
    case OMD_INTERPOLATED_POSITION_MODE:
      VCS_StopIpmTrajectory(controller, nodeId, &errorCode);
      break;
    case OMD_POSITION_MODE:
    case OMD_VELOCITY_MODE:
    case OMD_CURRENT_MODE:
    case OMD_MASTER_ENCODER_MODE:
    case OMD_STEP_DIRECTION_MODE:
      break;
  }

  stopEndTime = bigss::time_now_ms();
}

void maxonMotor::getNegativeLimit(mtsDouble &limit) const
{
  limit = negLimit;
}

void maxonMotor::getPositiveLimit(mtsDouble &limit) const
{
  limit = posLimit;
}

void maxonMotor::setNegativeLimit(const mtsDouble &limit)
{
  negLimit = limit;
  posLimit = negLimit + stroke;
}

void maxonMotor::setNegativeLimitCurrentPosition()
{
  this->setNegativeLimit(position);
}

void maxonMotor::setPositiveLimit(const mtsDouble &limit)
{
  posLimit = limit;
  negLimit = posLimit - stroke;
}

void maxonMotor::setPositiveLimitCurrentPosition()
{
  this->setPositiveLimit(position);
}

void maxonMotor::setSetPointMode(const mtsBool &b)
{
  CMN_LOG_CLASS_RUN_DEBUG << "setting set point mode to " << b.Data << std::endl;
  setPointMode = b.Data;
}

void maxonMotor::setVelocity(const mtsDouble &vel)
{
  setVelocityStartTime = bigss::time_now_ms();

  targetVelocity = std::abs(unitsToRPM(vel.Data));
  setPositionProfile();

  setVelocityEndTime = bigss::time_now_ms();
}

void maxonMotor::setVelocity_raw(const mtsLong &vel)
{
  targetVelocity = std::abs(vel.Data);
  setPositionProfile();
}

void maxonMotor::setVelocityPoint(const mtsDouble &vel)
{
  lastVelocityCommand = bigss::time_now_ms();
  CMN_LOG_CLASS_RUN_DEBUG << "setting velocity point to " << vel.Data
    << " mm" << std::endl;
  velocitySetPoint = unitsToRPM(vel.Data);
}

void maxonMotor::setVelocityPoint_raw(const mtsDouble &vel)
{
  lastVelocityCommand = bigss::time_now_ms();
  CMN_LOG_CLASS_RUN_DEBUG << "setting velocity point to "
    << vel.Data << " rpm" << std::endl;
  velocitySetPoint = vel.Data;
}

// raw is always in rpm
void maxonMotor::getTargetVelocity_raw(mtsLong &vel)
{
  vel = targetVelocity;
}

void maxonMotor::getTargetVelocity(mtsDouble &vel)
{
  vel = rpmToUnits(targetVelocity);
}

void maxonMotor::setMaxVelocity(const double vel)
{
  unsigned int maxVel = static_cast<unsigned int>(unitsToRPM(vel));
  VCS_SetMaxProfileVelocity(controller, nodeId, maxVel, &errorCode);
}

void maxonMotor::setMaxVelocity_raw(const unsigned int maxVel)
{
  VCS_SetMaxProfileVelocity(controller, nodeId, maxVel, &errorCode);
}

double maxonMotor::getMaxVelocity()
{
  DWORD maxVel;
  VCS_GetMaxProfileVelocity(controller, nodeId, &maxVel, &errorCode);
  return rpmToUnits(maxVel);
}

void maxonMotor::setMaxAcceleration(const double acc)
{
  // convert to rpm/s from mm/s2
  DWORD maxAcc = static_cast<DWORD>(unitsToRPM(acc));
  VCS_SetMaxAcceleration(controller, nodeId, maxAcc, &errorCode);
}

void maxonMotor::setMaxAcceleration_raw(const unsigned int acc)
{
  VCS_SetMaxAcceleration(controller, nodeId, acc, &errorCode);
}

double maxonMotor::getMaxAcceleration()
{
  DWORD maxAcc;
  VCS_GetMaxAcceleration(controller, nodeId, &maxAcc, &errorCode);
  // convert to mm/min/sec and then to mm/sec2
  return rpmToUnits(maxAcc);
}

void maxonMotor::setPositionMode()
{
  stop();
  operationalMode = OMD_PROFILE_POSITION_MODE;
  CMN_LOG_CLASS_RUN_DEBUG << "Setting position mode" << std::endl;
  VCS_SetOperationMode(controller, nodeId, OMD_PROFILE_POSITION_MODE, &errorCode);
}

void maxonMotor::setVelocityMode()
{
  stop();
  operationalMode = OMD_PROFILE_VELOCITY_MODE;
  VCS_SetOperationMode(controller, nodeId, OMD_PROFILE_VELOCITY_MODE, &errorCode);
}

bool maxonMotor::getVelocityProfile()
{
  return VCS_GetVelocityProfile(controller, nodeId, &accel, &decel, &errorCode) != 0;
}

bool maxonMotor::setVelocityProfile()
{
  return VCS_SetVelocityProfile(controller, nodeId, accel, decel, &errorCode) != 0;
}

bool maxonMotor::getPositionProfile()
{
  DWORD vel;
  return VCS_GetPositionProfile(controller, nodeId, &vel, &accel, &decel, &errorCode) != 0;
}

bool maxonMotor::setPositionProfile()
{
  DWORD vel = std::abs(targetVelocity);
  return VCS_SetPositionProfile(controller, nodeId, vel, accel, decel, &errorCode) != 0;
}

void maxonMotor::moveToAbsolutePosition(const mtsDouble & pos)
{
  moveToPosition(pos.GetData(), true);
}

void maxonMotor::moveToAbsolutePosition_raw(const mtsLong & pos)
{
  moveToPosition_raw(pos.GetData(), true);
}

void maxonMotor::moveToRelativePosition(const mtsDouble & pos)
{
  moveToPosition(pos.GetData(), false);
}

void maxonMotor::moveToRelativePosition_raw(const mtsLong & pos)
{
  moveToPosition_raw(pos.GetData(), false);
}

bool maxonMotor::moveToPosition(const double pos, const bool abs, const bool immediate)
{
  long posl = unitsToEncoderCounts(pos);
  return moveToPosition_raw(posl, abs, immediate);
}

bool maxonMotor::moveToPosition_raw(const long pos, const bool abs, const bool immediate)
{
  long posl = pos;
  if(abs)
    posl += zeroPos;
  return VCS_MoveToPosition(controller, nodeId, posl, abs, immediate, &errorCode) != 0;
}

void maxonMotor::jogPlus()
{
  CMN_LOG_CLASS_RUN_VERBOSE << "moving motor " << this->Name << " with positive velocity " << targetVelocity << std::endl;
  moveVelocityStartTime = bigss::time_now_ms();
  VCS_MoveWithVelocity(controller, nodeId, targetVelocity, &errorCode);
  moveVelocityEndTime = bigss::time_now_ms();
}

void maxonMotor::jogMinus()
{
  CMN_LOG_CLASS_RUN_VERBOSE << "moving motor " << this->Name << " with negative velocity " << targetVelocity << std::endl;
  VCS_MoveWithVelocity(controller, nodeId, -targetVelocity, &errorCode);
}

void maxonMotor::getErrorString(mtsStdString &errorStr)
{
  char error[100];
  errorStr.Data = "No error";

  if(VCS_GetErrorInfo(errorCode, error, 100))
    errorStr.Data = std::string(error);
}

inline double maxonMotor::encoderCountsToUnits(const long &encCounts) const
{
  return (double)encCounts * pitch / (gearRatio * countsPerTurn * quadrature);
}

inline long maxonMotor::unitsToEncoderCounts(const double &units) const
{
  return static_cast<long>(units * (countsPerTurn * quadrature * gearRatio) / (pitch));
}

inline long maxonMotor::unitsToRPM(const double &units) const
{
  return static_cast<long>(units * gearRatio / (pitch) * 60.0);
}

inline double maxonMotor::rpmToUnits(const long &rpm) const
{
  return (double)rpm * (pitch) / (60.0 * gearRatio);
}

inline double maxonMotor::aiToVolts(const WORD &ai) const
{
  return (double)ai * aiCal;
}

inline double maxonMotor::voltsToForce (const double &volts) const
{
  return volts * forceGain;
}
