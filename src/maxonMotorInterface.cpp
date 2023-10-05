#include "maxonMotorInterface.h"

#include <cisstMultiTask/mtsInterfaceRequired.h>

maxonMotorInterface::maxonMotorInterface(mtsInterfaceRequired *requiredControl, mtsInterfaceRequired *requiredState)
{
  this->configureControlInterface(requiredControl);
  this->configureStateInterface(requiredState);
}

void maxonMotorInterface::configureControlInterface(mtsInterfaceRequired *required)
{
  required->AddFunction("enable", enable);
  required->AddFunction("disable", disable);
  required->AddFunction("clearFault", clearFault);
  required->AddFunction("stop", stop);
  required->AddFunction("setVelocity", setVelocity);
  required->AddFunction("setVelocity_raw", setVelocity_raw);
  required->AddFunction("setVelocityPoint", setVelocityPoint);
  required->AddFunction("setVelocityPoint_raw", setVelocityPoint_raw);
  required->AddFunction("setSetPointMode", setSetPointMode);
  required->AddFunction("getTargetVelocity", getTargetVelocity);
  required->AddFunction("getTargetVelocity_raw", getTargetVelocity_raw);
  required->AddFunction("jogPlus", jogPlus);
  required->AddFunction("jogMinus", jogMinus);
  required->AddFunction("moveToAbsolutePosition", moveToAbsolutePosition);
  required->AddFunction("moveToRelativePosition", moveToRelativePosition);
  required->AddFunction("moveToAbsolutePosition_raw", moveToAbsolutePosition_raw);
  required->AddFunction("moveToRelativePosition_raw", moveToRelativePosition_raw);
  required->AddFunction("setPositionMode", setPositionMode);
  required->AddFunction("setVelocityMode", setVelocityMode);
  required->AddFunction("getErrorString", getErrorString);
  required->AddFunction("getNodeId", getNodeId);
  required->AddFunction("zero", zero);
  required->AddFunction("moveToZero", moveToZero);
  required->AddFunction("rebias", rebias);
  required->AddFunction("setHomeForce", setHomeForce);
  required->AddFunction("startForceHoming", startForceHoming);
  required->AddFunction("setDigitalOutput", setDigitalOutput);
  required->AddFunction("negativeLimit", negativeLimit);
  required->AddFunction("positiveLimit", positiveLimit);
  required->AddFunction("setNegativeLimit", setNegativeLimit);
  required->AddFunction("setPositiveLimit", setPositiveLimit);
  required->AddFunction("setNegativeLimitCurrentPosition", setNegativeLimitCurrentPosition);
  required->AddFunction("setPositiveLimitCurrentPosition", setPositiveLimitCurrentPosition);
}

void maxonMotorInterface::configureStateInterface(mtsInterfaceRequired *required)
{
  required->AddFunction("operationMode", operationMode);
  required->AddFunction("enabled", enabled);
  required->AddFunction("position_raw", position_raw);
  required->AddFunction("velocity_raw", velocity_raw);
  required->AddFunction("velocityAverage_raw", velocityAverage_raw);
  required->AddFunction("position", position);
  required->AddFunction("velocity", velocity);
  required->AddFunction("velocityAverage", velocityAverage);
  required->AddFunction("current", current);
  required->AddFunction("currentAverage", currentAverage);
  required->AddFunction("fault", fault);
  required->AddFunction("time", time);
  required->AddFunction("force", force);
}

