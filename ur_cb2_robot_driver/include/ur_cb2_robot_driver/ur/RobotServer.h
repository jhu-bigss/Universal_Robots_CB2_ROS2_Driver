#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <cisstOSAbstraction/osaSocketServer.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>

#include "RobotInterface.h"

/**
 * @brief Class that enables sending/receiving commands with programs running on the UR
 * This class intializes a mtsTaskPeriodic and then establishes two network connections
 * to the actual robot. The first connection should occur before the robot task is started
 * and will enable the uploading of a program onto the UR. When the program is uploaded,
 * the second connection is established, allowing for communication with the BIGSS UR
 * server program or programs with a similar API.
 */

class RobotServer
{
  // attributes
protected:

  osaSocketServer server;
  osaSocket *progSocket;

  bool isConnectedToProg;  ///< Is the program currently running on the UR connected to this server
  unsigned short host_port;  ///< The port on this computer that the server will listen for connections from programs running on the UR

  // Merging in fields from robotInterface
  osaSocket urSocket;  ///< socket used to read information on the state of the UR and also send UR Script programs; we are connecting as a client to the built-in server running on the UR
  bool isConnectedToUR;  ///< Have we connected to the UR server as a client

  URDHKinematics ur_kins_; ///< performs ur kinematics calculations using preset dh parameters
  bool use_high_level_pd_; ///< not sure what this does

  prmPositionJointGet JointPos;
  prmPositionCartesianGet CartPos;
  prmVelocityJointGet JointVel;
  prmVelocityCartesianGet CartVel;
  vctDoubleMat GeoJacobian;

  bool velocityTimeLimit;
  int lastVelocityCommandTime;
  int lastVelocityCommandThresh;
  vctDoubleVec lastVelocityCommand;

  RobotInterface ur_interface; ///< converts ur packets from socket to state data
  /// The robot trajectory to follow
  //std::vector< vctFrm4x4 > robotTraj;   //< The cartesian trajectory
  vctDoubleMat jointTraj;               //< The joint trajectory, a 6 x n matrix
  double trajTolerance;                 //< When (currentJoints - waypointJoints).Norm() < trajTolerance, move to next
  int trajID;                           //< Current ID in the joint trajectory that is trying to be reached. 
  double trajVel;                       //< Maximum trajectory velocity
  bool commandingTraj;                  //< If true, then robot is commanding a trajectory

  enum Mode {
    SPATIAL_POSITION = 0,   // Move the UR to a cartesian position using built-in commands
    JOINT_POSITION = 1,         // Move the UR joint positions using build-in commands
    SPATIAL_VELOCITY = 2,       // Move the UR with a spatial velocity
    JOINT_VELOCITY = 3,         // Move the UR with a joint velocity
    STOP_JOINT_SPACE = 4,       // Stop movement linear in joint space
    SET_SPEED_PERCENT = 5,      // Set the percent speed for movej & movel commands.
    INVERSE_KINEMATICS = 6,     // Get IK of robot [not yet implemented]
    PD_JOINT_SPACE = 7,         // PD control in joint space, move with velocity
    PD_CARTESIAN = 8,           // PD control to reach position, move with velocity
    SET_PD_SPEED = 9,           // Set the max PD speed of the robot
  };

  // these are taken from Dr. Kazanzides' C-API UR5 code

  void GetJointModes(vctInt6 &modes) const;

  // Returns a boolean vector indicating whether the corresponding joint is homed (initialized)
  void IsJointHomed(vctBool6 &stateVec) const;

  // Returns true if all joints homed; false otherwise
  void IsHomed(bool &state) const;

  // Returns true if all joints homed; false otherwise
  bool IsHomed(void) const { bool homed; IsHomed(homed); return homed; }

  // Returns true if motor power is on; false otherwise
  void IsMotorPowerOn(bool &state) const;

  // Returns true if e-stop activated
  void IsEStop(bool &state) const;

  // Return true if security stop activated (software-controlled stop)
  void IsSecurityStop(bool &state) const;

  // Set Robot Modes
  void SetRobotFreeDriveMode(void);

  void SetRobotRunningMode(void);

  // Stop Motion
  void StopMotion(void);

  // Given a joint configuration (radians), return the robot pose and Jacobian
  void FwdKin(const vctDoubleVec& joints, vctFrm3& pose, vctDoubleMat& jac);

public:

  RobotServer(const std::string &name = "RobotServer", const unsigned short &sp=50000);
  ~RobotServer();

  void AssignServerPort(const unsigned short port);

  void SetURType(const std::string &str);

  void SendToURClient(const mtsStdString &str);

  // Move joint to specified position (radians)
  void SetPositionJoint(const vctDoubleVec& joints);
  void SetPositionJoint(const std::array<double, 6>& joint_pos);

  // Move joint at specified velocity (radians/sec)
  void SetVelocityJoint(const vctDoubleVec& joints_vels);
  void SetVelocityJoint(const std::array<double, 6>& joints_vel);

  // Get the current joint velocities (radians/sec)
  void UpdateVelocityJoint(vctDoubleVec& joints_vels) const;

  // Move to the desired EEF pose
  void SetPositionCartesian(const vctFrm3& pose);

  // Get the current EEF pose
  void UpdatePositionCartesian(vctFrm3& pose) const;

  // Move at the specified velocity
  void SetVelocityCartesian(const vctDoubleVec& vel);

  // Retreives the current velocity of the EEF
  // This needs to be retrieved via UR Script
  void UpdateVelocityCartesian(vctDoubleVec& vel) const;

  void GetGeoJacobian(vctDoubleMat& j) const;

  void GetRobotState(UniversalRobot::RobotState &s) const;

  // Given a desired robot pose, return the joint configuration nearest to the
  // current configuration
  void InvKin(const vctFrm3& pose, vctDoubleVec& joints,
    const vctDoubleVec &initalJoints = vctDoubleVec(6, 0.0));

  void SetUseHighLevelPDControl(const bool& b);

  /// Set the percentage of max speed to use in the PD control
  /// This allows users to have a bit more control about how fast
  /// the robot moves.
  void SetPDSpeedPercent(const double& value);

  void SetSpeedPercent(const mtsInt &value);

  void GetUseHighLevelPDControl(bool& b);

  bool ConnectToUR(const std::string& ur_host, unsigned short ur_port=30002);
  void SetupURProgSocket();

    /*!
   * \brief Program the robot from a file
   * \param filename The file to read and send
   */
  void ProgramFromFile(const std::string &filename);

  /// Send a program to the UR
  /// \param program The program to send
  /// \param acceptSocket If true (default), try to accept a comm socket from the UR to pass data over.
  void SendProgramToUR(const std::string &program, bool acceptSocket = true);

  /// \brief Sets a cartesian trajectory for the end-effector of the robot
  /// \param poses The poses of the robot eef wrt the robot base. Translation in meters
  /// \param vel The constant velocity for the trajectory
  void CommandCartesianTrajectory (const std::vector <vctFrm4x4> &poses, const double vel);

private:

   /// Send a vector of doubles to the UR5.
  /// param mode The command identifier
  /// param v The vector of values
  /// param pad If true, pad to a 6-length vector [true]
  void SendDoubleVec(const unsigned long mode, const vctDoubleVec& v, const bool pad = true);

  /// Read doubles from the socket
  /// param n The number of doubles to read
  /// param doubles The values read
  /// this is not being used currently!!!
  void ReadDoubles(const unsigned int n, vctDoubleVec &doubles);
  
  /// Check if trajectory position should update
  /// \return True if the translation components is within 0.5mm.
  bool CheckTrajUpdate ();

  /// Step to the next trajectory target
  void StepTraj();

  static void ConvertFrm3ToURPose(const vctFrm3 vct_pose, vctDoubleVec& ur_pose);

  std::string ur_network_id;

};

#endif // ROBOT_SERVER_H

