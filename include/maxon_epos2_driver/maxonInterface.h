#ifndef _MAXON_INTERFACE_H
#define _MAXON_INTERFACE_H

#include <string>
#include <vector>

#include <cisstVector/vctDynamicVector.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsComponent.h>

class maxonMotor;
struct maxonMotorInterface;

#ifndef WIN32
typedef void* HANDLE;
typedef unsigned int DWORD;
typedef unsigned short WORD;
#else
  #ifndef _WINDOWS_
    #include <windows.h>
  #endif
#endif

class maxonInterface : public mtsComponent
{
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)
  
  // attributes
public:

protected:
  HANDLE controller; //< main controller
  DWORD errorCode; //< error code from calls to library

  ///////////
  // Comms //
  ///////////
  std::string protocolStack;
  std::string ifaceName;
  std::string devName;
  std::string portName;
  int baudrate; //< baudrate

  //std::vector< maxonMotor * > motors; //< attached can nodes

  std::vector< mtsCollectorState *> collectors; //< data collectors for each motor

  bool connected; //< is the controller connected

  std::vector< maxonMotorInterface *> motorInterfaces;
  std::vector< maxonMotor *> motorTasks;

  // methods
public:
  maxonInterface(const std::string &devName, const std::string &portName = "USB0");
  ~maxonInterface();

  /// Startup function called by the task manager
  /// This starts collectors as necessary
  virtual void Startup();

  /// Cleanup function called by the task manager
  /// This stops collectors as necsesary
  virtual void Cleanup();

  /// Load configuration from a file
  /// The configuration file is a json file. See examples.
  /// This should be called after initialization
  bool addNodes(const std::string &filename);

  /// Connect the client interfaces with the motor task interfaces
  /// via a given local manager
  void Connect(mtsManagerLocal *localManager);

  /// Add a node to the CAN bus
  /// \param id The node id
  /// \param filename The configuration file
  /// \param taskname The taskname
  /// \param loadCellCal The load cell calibration
  void addNode(const unsigned short id, const std::string &filename, const std::string &taskname, const double &loadCellCal);
  
  /// Get the ith motor added
  /// \return The motor, if it exists. Otherwise 0.
  maxonMotor *getMotor(const unsigned int i);

  maxonMotorInterface *getMotorInterface(const unsigned int i);
  std::vector<std::string> getMotorList();

  /// Return the number of motors
  size_t getNumMotors() {return motorInterfaces.size();};

  /// Create the MTS collector objects
  /// This should be called before starting the task so that the collectors may be started
  void createCollectors();
  void addCollectors(mtsManagerLocal *componentManager);

  void startCollection();
  void stopCollection();

  /// Returns true if the motion target has been reached by all motors
  bool targetReached();

  ////////////////////
  // Motor commands //
  ////////////////////

  /// Initialize the communication
  bool initialize(/*int i=0*/);
  bool close();

  /// Disable all the motors
  void disable();

  /// Set the velocity
  /// \param vel The velocity (mm/s)
  void setVelocity(const double vel);
  void setVelocity(const unsigned int &motorId, const double vel);

  /// Set the velocity
  /// \param vel The velocity (rpm)
  void setVelocity_raw(const long vel);
  void setVelocity_raw(const unsigned int &motorId, const long &vel);

  /// Move to the target position of each motor
  /// \param pos A double vector of positions
  /// \param abs If false (default), perform a relative movement. Otherwise, move to the absolute position
  void moveToPosition(const double *pos, const bool abs = false);
  void moveToPosition(std::vector<double> pos, const bool abs = false);
  void moveToPosition(const vctDoubleVec &pos, const bool abs = false);
  void moveToPosition(const unsigned int &motorId, const double &pos, const bool abs = false);

  /// Get the current positions
  void getPositions(std::vector<double> &pos);
  void getPositions(vctDoubleVec &pos);

  /// Get the negative limits
  void getNegativeLimits(vctDoubleVec &limits);
  void getPositiveLimits(vctDoubleVec &limits);

  /// Get the forces measured by the motors
  void getForce(vctDoubleVec &force);

  /// Get error information
  std::string getErrorString();
  
  /// Return true if the EPOS2 is connected
  bool isConnected() const {return connected;};

  /// Sets digital output pins of the controllers
  void setDigitalOutput(const std::vector<int> &vals);

  void setVelocityMode();

  void printCommunicationSelections();

protected:
  /// Perform MTS task setup for the motor
  void connectMotor(maxonMotor *m);

};

CMN_DECLARE_SERVICES_INSTANTIATION(maxonInterface)

#endif // _MAXON_INTERFACE_

