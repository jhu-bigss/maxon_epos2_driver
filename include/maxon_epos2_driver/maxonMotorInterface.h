#ifndef _MAXON_MOTOR_INTERFACE_H
#define _MAXON_MOTOR_INTERFACE_H

#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstMultiTask/mtsFunctionVoid.h>
#include <cisstMultiTask/mtsFunctionVoidReturn.h>

/////////////////////////
// maxonMotorInterface //
/////////////////////////
/*!
 * This struct serves to encapsulate the MTS structure for the maxon motor here, making
 * things a bit easier internally
 */
struct maxonMotorInterface
{
  maxonMotorInterface(mtsInterfaceRequired *controlInterface, mtsInterfaceRequired *stateInterface);
  maxonMotorInterface() {};
  ~maxonMotorInterface() {};

  void configureControlInterface(mtsInterfaceRequired *required);
  void configureStateInterface(mtsInterfaceRequired *required);

  // required control functions
  mtsFunctionVoidReturn enable;
  mtsFunctionVoid disable;
  mtsFunctionVoid clearFault;
  mtsFunctionVoid stop;
  mtsFunctionVoidReturn getTargetVelocity;
  mtsFunctionVoidReturn getTargetVelocity_raw;
  mtsFunctionWrite setVelocity;
  mtsFunctionWrite setVelocity_raw;
  mtsFunctionWrite setSetPointMode;
  mtsFunctionWrite setVelocityPoint;
  mtsFunctionWrite setVelocityPoint_raw;
  mtsFunctionVoid jogPlus;
  mtsFunctionVoid jogMinus;
  mtsFunctionWrite moveToAbsolutePosition;
  mtsFunctionWrite moveToAbsolutePosition_raw;
  mtsFunctionWrite moveToRelativePosition;
  mtsFunctionWrite moveToRelativePosition_raw;
  mtsFunctionVoid setPositionMode;
  mtsFunctionVoid setVelocityMode;
  mtsFunctionVoidReturn getErrorString;
  mtsFunctionRead getNodeId;
  mtsFunctionVoid zero;
  mtsFunctionVoid moveToZero;
  mtsFunctionRead negativeLimit;
  mtsFunctionRead positiveLimit;
  mtsFunctionWrite setNegativeLimit;
  mtsFunctionWrite setDigitalOutput;
  mtsFunctionWrite setPositiveLimit;
  mtsFunctionVoid setNegativeLimitCurrentPosition;
  mtsFunctionVoid setPositiveLimitCurrentPosition;
  mtsFunctionVoid rebias;
  mtsFunctionWrite setHomeForce;
  mtsFunctionVoid startForceHoming;
  mtsFunctionRead getCableForce;

  // required state functions
  mtsFunctionRead operationMode;
  mtsFunctionRead enabled;
  mtsFunctionRead position_raw;
  mtsFunctionRead velocity_raw;
  mtsFunctionRead velocityAverage_raw;
  mtsFunctionRead position;
  mtsFunctionRead velocity;
  mtsFunctionRead velocityAverage;
  mtsFunctionRead current;
  mtsFunctionRead currentAverage;
  mtsFunctionRead fault;
  mtsFunctionRead time;
  mtsFunctionRead force;
};

#endif

