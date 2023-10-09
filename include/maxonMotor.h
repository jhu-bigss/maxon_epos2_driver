#ifndef _MAXON_MOTOR_H
#define _MAXON_MOTOR_H

#include <string>
#include <cisstMultiTask/mtsTaskPeriodic.h>

// enable cross-platform usage
#ifndef WIN32
typedef void* HANDLE;
typedef unsigned int DWORD;
typedef unsigned short WORD;
#else
  #ifndef _WINDOWS_
    #include <windows.h>
  #endif
#endif

/*!
 * This class defines a single maxon motor controller. Each controller is connected either to another motor controller
 * on the CAN bus or to the PC via USB. The PC comms are defined in maxonInterface.
 *
 */
class maxonMotor : public mtsTaskPeriodic
{
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)

  // attributes
public:

  struct status {
    std::string operationMode;
#ifdef WIN32
    long position_raw;          // encoder counts
    long velocity_raw;          // rpm
    long velocityAverage_raw;   // rpm
#else
    int position_raw;           // encoder counts
    int velocity_raw;           // rpm
    int velocityAverage_raw;    // rpm
#endif
    double position;	        // [mm or deg]
    double velocity;            // [units/s]
    double velocityAverage;     // [units/s]
    short current;
    short currentAverage;
    double ai_raw;              // volts
    double force;               // lbs
  };

protected:

  HANDLE controller; //< main controller
  DWORD errorCode;  //< error code from calls to library

  unsigned short nodeId; //< CAN Node id for motor

  DWORD accel;  //< profile acceleration
  DWORD decel;  //< profile deceleration
  long zeroPos; //< the zero position

  long targetVelocity; //< target velocity for movement. Internally, this is always kept as a positive (even though it's not unsigned)
  double velocitySetPoint;

  /// conversion between mm/deg and encoder
  double pitch;      //< Screw pitch, in inch per thread. Set to 1 if there is no lead screw
  double gearRatio;  //< Gear ratio
  int quadrature;    //< Quadrature
  int countsPerTurn; //< Encoder counts per turn

  ////////////
  // Limits //
  ////////////
  // Note: There are no software stops for the limits as of 12 Aug 2015.
  double negLimit; //< The negative limit of the motor, relative to the zero position
  double posLimit; //< The positive limit of the motor, relative to the zero position [automatically set with the negative limit based on the stroke]
  double stroke;   //< The maximum travel of the motor, in mm

  ////////////
  // Status //
  ////////////
  mtsBool enabled;       //< is the motor enabled?
  mtsBool fault;         //< is the motor in a fault status?
  mtsBool targetReached; //< has the movement target been reached?

  mtsStdString time;

  //////////////////
  // Force Homing //
  //////////////////
  mtsBool homing;
  double homingDesForce;
  double homingThresh;

  //////////////////
  // Analog input //
  //////////////////
  bool hasAI;          //< Set to true if there is a differential AI on pins 1 and 2.
  double aiCal;        //< Analog input calibration [5 V at 2^12 resolution]
  double forceGain;    //< Force gain to convert from volts to pounds
  double analogInput_bias;  // sensor bias

  /// The operational mode
  /// See Definitions.h for the values
  char operationalMode;
  mtsStdString operationMode;
#ifdef WIN32
  mtsLong position_raw;          // encoder counts
  mtsLong velocity_raw;          // rpm
  mtsLong velocityAverage_raw;   // rpm
#else
  mtsInt position_raw;           // encoder counts
  mtsInt velocity_raw;           // rpm
  mtsInt velocityAverage_raw;    // rpm
#endif
  mtsDouble position;	           // [mm or deg]
  mtsDouble velocity;            // [units/s]
  mtsDouble velocityAverage;     // [units/s]
  mtsShort current;
  mtsShort currentAverage;
  mtsDouble analogInput_raw;     // [V] Assumes differential analog input with (+) on AI1 and (-) on AI2
  mtsDouble force;               // [lbs, or whatever cal units are]

  size_t numCommandsProcessed;
  double startTime;
  double endTime;
  double setVelocityStartTime;
  double setVelocityEndTime;
  double moveVelocityStartTime;
  double moveVelocityEndTime;
  double stopStartTime;
  double stopEndTime;

  bool setPointMode;
  int lastVelocityCommand;
  int lastVelocityCommandThresh;

  // methods
public:
  /// Constructor
  /// \param id The CAN node id
  maxonMotor(const unsigned short id, HANDLE controller, const std::string &taskName);
  maxonMotor(const unsigned short id, const std::string &filename, HANDLE controller, const std::string &taskName);
  ~maxonMotor();

  /// Set the controller
  void setController(HANDLE c) {controller = c;};

  /// Load the motor parameters
  /// \param filename The json file
  bool loadMotorConfiguration(const std::string &filename);

  /// Get the node id
  void getNodeId(mtsUShort &id) const;

  /// Clear any fault states
  void clearFault();

  /// Set whether or not the AI should be used.
  /// When AI is used, we assume it is for a load cell and compute the differential
  /// input with (+) on AI1 and (-) on AI2.
  void setUseAI(bool useAI) {
    hasAI = useAI;
  }

  bool getUseAI() const {
    return hasAI;
  }


  /// Get the limits
  double getNegativeLimit() const {return negLimit;};
  void getNegativeLimit(mtsDouble &limit) const;
  double getPositiveLimit() const {return posLimit;};
  void getPositiveLimit(mtsDouble &limit) const;

  /// Set the limits
  /// Setting the negative limit will automatically update the positive
  /// limit based on the stroke.
  void setNegativeLimit(const mtsDouble &limit);
  void setPositiveLimit(const mtsDouble &limit);

  /// Set the limits from the current position.
  /// Setting the negative limit will automatically update the positive
  /// limit based on the stroke.
  void setNegativeLimitCurrentPosition();
  void setPositiveLimitCurrentPosition();

  /// Set the target velocity
  void setVelocity(const mtsDouble &vel);
  void setVelocity_raw(const mtsLong &vel);
  void setVelocityPoint(const mtsDouble &vel);
  void setVelocityPoint_raw(const mtsDouble &vel);

  /// Get the target velocity
  void getTargetVelocity(mtsDouble &vel);
  void getTargetVelocity_raw(mtsLong &vel);

//  void setAccel();
//  DWORD getAccel() const {return accel;};
//
//  void setDecel();
//  DWORD getDecel() const {return decel;};

  /// Max velocity
  /// \param vel The max velocity, in rpm
  void setMaxVelocity_raw(const unsigned int vel);

  /// Max velocity
  /// \param vel The max velocity, in mm/s
  void setMaxVelocity(const double vel);
  double getMaxVelocity();

  /// Set max acceleration
  /// \param acc The acceleration, in rpm/s
  void setMaxAcceleration_raw(const unsigned int acc);

  /// Set max acceleration
  /// \param acc The acceleration, in mm/s2
  void setMaxAcceleration(const double acc);
  double getMaxAcceleration();

  void setPitch(const double pitch_) {
    pitch = pitch_;
  };
  double getPitch() const {
    return pitch;
  };

  void setGearRatio(const double ratio) {
    gearRatio = ratio;
  };
  double getGearRatio() const {
    return gearRatio;
  };

  void setQuadrature(const int quad) {
    quadrature = quad;
  };
  int getQuadrature() const {
    return quadrature;
  };

  void setCountsPerTurn(int counts) {
    countsPerTurn = counts;
  };
  int getCountsPerTurn() {
    return countsPerTurn;
  };

  void setMotorParams(double pitch_, double gearRatio_,
      int quadrature_, int countsPerTurn_) {
    pitch = pitch_;
    gearRatio = gearRatio_;
    quadrature = quadrature_;
    countsPerTurn = countsPerTurn_;
  }

  void setForceGain(const double &gain) {
    forceGain = gain;
  }

  void setHomeForce(const mtsDouble &homeForce);
  void startForceHoming();
  void checkForceHoming();

  void rebias();

  /// Get the status of the motor
  /// \param state The structure to fill in
  /// \return True if valid
  bool getStatus(struct status &stat);

  /// Enable the motor
  void enable(mtsBool &result);
  void disable();

  /// Zero the position
  void zero();

  ///////////////////////
  /// Motion control ///
  //////////////////////
  void stop();
  void setSetPointMode(const mtsBool &b);
  void setPositionMode();
  void setVelocityMode();

  /// Move to the zero position
  void moveToZero();

  /// Jog the motor in the positive direction
  /// Users must set the velocity prior to this call
  void jogPlus();

  /// Jog the motor in the negative direction
  /// Users must set the velocity prior to this call
  void jogMinus();

  /// Move to an absolute position
  /// \param pos The position (mm or deg)
  void moveToAbsolutePosition(const mtsDouble & pos);

  /// Move to a position
  /// \param pos The position (encoder counts)
  void moveToAbsolutePosition_raw(const mtsLong & pos);

  /// Move to a relative position
  /// \param pos The position (mm or deg)
  void moveToRelativePosition(const mtsDouble & pos);

  /// Move to a relative position
  /// \param pos The position (encoder counts)
  void moveToRelativePosition_raw(const mtsLong & pos);

  /// Get error information
  void getErrorString(mtsStdString &errorString);

  // Sets digital outputs
  void setDigitalOutput(const mtsInt &val);

  // resets timing vars to default values
  void resetTimingVariables();

  ///////////////
  // MTS Tasks //
  ///////////////
  virtual void Startup();
  virtual void Cleanup();
  virtual void Configure(const std::string &filename);
  virtual void Run();

protected:
  bool getVelocityProfile();
  bool getPositionProfile();
  bool setVelocityProfile();
  bool setPositionProfile();

  /// Move to a position
  /// \param pos The position (mm or deg)
  /// \param abs If false, perform a relative move. Otherwise, perform an absolute move.
  /// \param immediate If true, move immediately. Otherwise, wait for previous move to stop
  bool moveToPosition(const double pos, const bool abs = false, const bool immediate = true);

  /// Move to a position
  /// \param pos The position (encoder counts)
  /// \param abs If false, perform a relative move. Otherwise, perform an absolute move.
  /// \param immediate If true, move immediately. Otherwise, wait for previous move to stop
  bool moveToPosition_raw(const long pos, const bool abs = false, const bool immediate = true);

  /////////////////
  // Conversions //
  /////////////////
  /// Convert from encoder counts to units (either mm or deg)
  /// \param encCounts the encoder counts
  /// \return The units (mm or deg, depending on configuration)
  inline double encoderCountsToUnits(const long &encCounts) const;

  /// Convert from units (either mm or deg) to encoder counts
  /// \param units The units (mm or deg, depending on configuration)
  /// \return the encoder counts
  inline long unitsToEncoderCounts(const double &units) const;

  /// Convert from units/second (mm or deg) to RPM
  /// \param unitsPerSecond
  /// \return RPM
  inline long unitsToRPM(const double &unitsPerSecond) const;

  /// Convert from units/second (mm or deg) to RPM
  /// \param RPM
  /// \return unitsPerSecond
  inline double rpmToUnits(const long &rpm) const;

  /// Convert from analog input values to Volts
  /// \param ai1 Analog input 1
  /// \return The voltage value
  inline double aiToVolts(const WORD &ai) const;

  /// Convert volts to force
  /// \param volts The measured voltage
  /// \param force The equivalent force
  inline double voltsToForce(const double &volts) const;

  /////////////////
  // MTS Helpers //
  /////////////////
  /// Create the MTS interface
  void createInterface();

  /// Update the status of the motor
  void updateStatus();
};

CMN_DECLARE_SERVICES_INSTANTIATION(maxonMotor)

#endif // _MAXON_MOTOR_H
