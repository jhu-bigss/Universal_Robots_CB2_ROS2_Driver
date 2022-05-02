#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

/*!
  This header file describes necessary structures and enumerations for the robot
  */

#ifndef _WIN32
#include <stdint.h>
typedef uint64_t ur_uint64;
#else
typedef unsigned __int64 ur_uint64;
#endif

namespace UniversalRobot
{
 enum PackageType {ROBOT_MODE_DATA = 0,
		    JOINT_DATA,
		    TOOL_DATA,
		    MASTERBOARD_DATA,
		    CARTESIAN_INFO,
		    KINEMATICS_INFO,
		    CONFIGURATION_DATA,
		    FORCE_MODE_DATA,
		    ADDITIONAL_INFO,
		    CALIBRATION_DATA};

#ifdef USE_UR_30
  enum RobotMode {ROBOT_DISCONNECTED = 0,
		  ROBOT_CONFIRM_SAFETY,
		  ROBOT_BOOTING,
		  ROBOT_POWER_OFF,
		  ROBOT_POWER_ON,
		  ROBOT_IDLE,
		  ROBOT_BACKDRIVE,
		  ROBOT_RUNNING};
#else
  enum RobotMode {ROBOT_RUNNING = 0,
    ROBOT_FREEDRIVE,
    ROBOT_READY,
    ROBOT_INITIALIZING,
    ROBOT_SERCURITY_STOPPED,
    ROBOT_EMERGENCY_STOPPED,
    ROBOT_FATAL_ERROR,
    ROBOT_NO_POWER,
    ROBOT_NOT_CONNECTED,
    ROBOT_SHUTDOWN,
    ROBOT_SAFEGUARD_STOP};
#endif

  enum JointMode {
#ifdef USE_UR_30
      SHUTTING_DOWN = 236,
#endif
		  PART_D_CALIBRATION = 237,
		  BACKDRIVE,
		  POWER_OFF,
		  EMERGENCY_STOPPED,
#ifndef USE_UR_30
      JOINT_ERROR,
      JOINT_FREEDRIVE,
      JOINT_SIMULATED,
#endif
		  NOT_RESPONDING = 245,
		  MOTOR_INITIALIZATION,
#ifdef USE_UR_30
      ADC_CALIBRATION,
#else
		  BOOTING,
#endif
		  DEAD_COMMUTATION,
		  BOOTLOADER,
		  CALIBRATION,
#ifndef USE_UR_30
      JOINT_STOPPED,
#endif
		  FAULT = 252,
		  RUNNING,
#ifndef USE_UR_30
      JOINT_INITIALIZATION,
#endif
		  IDLE = 255};

#ifdef USE_UR_30
  enum ToolMode {TOOL_BOOTLOADER = 249,
    TOOL_RUNNING = 253,
    TOOL_IDLE = 255};
#else
  enum ToolMode {TOOL_BOOTLOADER = 249,
    TOOL_RUNNING,
    TOOL_IDLE};
#endif

  enum MasterSafetyState {UNDEFINED = 0,
			  NORMAL,
			  REDUCED,
			  PROTECTIVE_STEP,
			  RECOVERY,
			  SAFEGUARD_STOP,
			  SYSTEM_EMERGENCY_STOP,
			  ROBOT_EMERGENCY_STOP,
			  VIOLATION,
			  MASTER_FAULT};

  enum MasterOnOffState {OFF = 0,
			 TURNING_ON,
			 ON,
			 TURNING_OFF};

  union intUnion {
    char raw[4];
    int data;
  };
  union doubleUnion {
    char raw[8];
    double data;
  };
  union floatUnion {
    char raw[4];
    float data;
  };
  union uint64Union {
    char raw[8];
    ur_uint64 data;
  };

#pragma pack(push, 1)
  struct RobotState {
    bool isRobotConnected;
    bool isRealRobotEnabled;
    bool isPowerOnRobot;
    bool isEmergencyStopped;
    bool isProtectedStopped;
    bool isProgramRunning;
    bool isProgramPaused;
    unsigned char robotMode;
#ifdef USE_UR_30
    unsigned char controlMode;
#endif
  };

  // 38 bytes
  struct RobotModeData {
    intUnion length;
    unsigned char type;
    uint64Union timestamp;
    struct RobotState state;
    doubleUnion targetSpeedFraction;
#ifdef USE_UR_30
    doubleUnion speedScaling;
#endif
  };

  struct SingleJoint {
    doubleUnion q_actual; // actual joint position
    doubleUnion q_target; // target joint position
    doubleUnion qd_actual; // actual joint speed
    floatUnion I_actual; // actual joint current
    floatUnion V_actual; // actual joint voltage
    floatUnion T_motor; // joint motor temperature

    floatUnion T_micro; // OBSOLETE
    unsigned char jointMode;
  };

  // 251 byes
  struct JointData {
    intUnion length;
    unsigned char type;
    struct SingleJoint joint[6];
  };

  // 37 bytes
  struct ToolData {
    intUnion length;
    unsigned char type;
    unsigned char aiRange2;
    unsigned char aiRange3;
    doubleUnion ai2;
    doubleUnion ai3;
    floatUnion toolVoltage;
    unsigned char toolOutputVoltage;
    floatUnion current;
    floatUnion temperature;
    unsigned char mode;
  };

  // 72 bytes
  struct MasterboardData {
    intUnion length;
    unsigned char type;
    intUnion digitalInputBits;
    intUnion digitalOutputBits;
    unsigned char analogInputRange0;
    unsigned char analogInputRange1;
    doubleUnion analogInput0;
    doubleUnion analogInput1;
    unsigned char analogOutputDomain0;
    unsigned char analogOutputDomain1;
    doubleUnion analogOutput0;
    doubleUnion analogOutput1;
    floatUnion temp;
    floatUnion robotVoltage;
    floatUnion robotCurrent;
    floatUnion masterIOCurrent;
    unsigned char safteyMode;
    unsigned char inReducedMode;
    unsigned char euromapInstalled;
    intUnion ignore;
  };

  // 88 bytes, only if euromap67 is installed
  struct MasterboardDataWithEuromap {
    intUnion length;
    unsigned char type;
    intUnion digitalInputBits;
    intUnion digitalOutputBits;
    unsigned char analogInputRange0;
    unsigned char analogInputRange1;
    doubleUnion analogInput0;
    doubleUnion analogInput1;
    unsigned char analogOutputDomain0;
    unsigned char analogOutputDomain1;
    doubleUnion analogOutput0;
    doubleUnion analogOutput1;
    floatUnion temp;
    floatUnion robotVoltage;
    floatUnion robotCurrent;
    floatUnion masterIOCurrent;
    unsigned char safteyMode;
    unsigned char inReducedMode;
    unsigned char euromapInstalled;
    intUnion euromapInput;
    intUnion euromapOutput;
    floatUnion euromapVoltage;
    floatUnion euromapCurrent;
    intUnion ignore;
  };

  // 53 bytes
  struct CartesianInfo {
    intUnion length;
    unsigned char type;
    // tool vector
    doubleUnion x;
    doubleUnion y;
    doubleUnion z;

    // tool rotation vector
    doubleUnion Rx;
    doubleUnion Ry;
    doubleUnion Rz;
  };

  struct JointLimits {
    doubleUnion jointMin;
    doubleUnion jointMax;
  };

  struct JointMovement {
    doubleUnion maxSpeed;
    doubleUnion maxAccel;
  };

  // 445 bytes
  struct ConfigurationData {
    intUnion length;
    unsigned char packageType;
    struct JointLimits jointLimits[6];
    struct JointMovement movement[6];

    // default joint speed/accel
    doubleUnion vJointDefault;
    doubleUnion aJointDefault;

    // default tool speed/accel
    doubleUnion vToolDefault;
    doubleUnion aToolDefault;

    // characteristic size of tool
    doubleUnion toolSize;

    // DH parameters
    doubleUnion dh_a[6];
    doubleUnion dh_d[6];
    doubleUnion dh_alpha[6];
    doubleUnion dh_theta[6];

    intUnion boardVersion;
    intUnion controllerBoxType;
    intUnion robotType;
    intUnion robotSubType;
  };

  // 61 bytes
  struct ForceModeData {
    intUnion length;
    unsigned char type;
    doubleUnion x; // force mode frame, x
    doubleUnion y; // force mode frame, y
    doubleUnion z; // force mode frame, z
    doubleUnion rX; // force mode frame, rx
    doubleUnion rY; // force mode frame, ry
    doubleUnion rZ; // force mode frame, rz
    doubleUnion dexterity; // tcp dexterity
  };

  // 7 bytes
  struct AdditionalInfo {
    intUnion length;
    unsigned char type;
    bool teachButtonPressed;
    bool teachButtonEnabled;
  };

  // 53 bytes
  struct CalibrationData {
    intUnion length;
    unsigned char type;
    unsigned char unused[48];
  };
#pragma pack(pop)

}  // namespace UniversalRobot


#endif // ROBOT_STATE_H

