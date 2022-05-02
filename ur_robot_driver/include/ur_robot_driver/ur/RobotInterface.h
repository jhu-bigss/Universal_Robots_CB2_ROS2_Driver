/*! \file robotInterface.h */

#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

//BTX

#include <cisstVector/vctTransformationTypes.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsFunctionWrite.h>

#include "URDHKinematics.h"
#include "RobotState.h"

// uncomment this if we are using 3.0 software.
// this should really be part of a CMake configuration
// This was a hack added in at the end after everything was written
// for 3.0 and we found we couldn't update.
//#define USE_UR_30

// Probably don't need these, we can just use sizeof calls
//#ifdef USE_UR_30
//#define ROBOT_MODE_SIZE 38
//#else
//#define ROBOT_MODE_SIZE 29
//#endif
//#define JOINT_DATA_SIZE 251
//#define TOOL_DATA_SIZE 37
//#define CARTESIAN_INFO_SIZE 53
//#define KINEMATICS_INFO_SIZE 225
//#define CONFIGURATION_DATA_SIZE 445
//#define FORCE_MODE_DATA_SIZE 61
//#define ADDITIONAL_INFO_SIZE 7
//#define CALIBRATION_DATA_SIZE 53

/*!
 * \class RobotInterface
 * \brief Network interface class to UR robot
 */
class RobotInterface
{
public:

  /*!
   * \union robotMode union
   * \brief Converts from raw packet to RobotModeData
   */
  union {
    char raw[sizeof(UniversalRobot::RobotModeData)];
    UniversalRobot::RobotModeData rmd;
  } robotMode;

  union {
    char raw[sizeof(UniversalRobot::JointData)];
    UniversalRobot::JointData jd;
  } jointData;

  union {
    char raw[sizeof(UniversalRobot::ToolData)];
    UniversalRobot::ToolData tool;
  } toolData;

  union {
    char raw[sizeof(UniversalRobot::MasterboardData)];
    UniversalRobot::MasterboardData data;
  } masterboardData;

  union {
    char raw[sizeof(UniversalRobot::MasterboardDataWithEuromap)];
    UniversalRobot::MasterboardDataWithEuromap data;
  } masterboardDataWithEuromap;

  union {
    char raw[sizeof(UniversalRobot::CartesianInfo)];
    UniversalRobot::CartesianInfo info;
  } cartesianInfo;

  union {
    char raw[sizeof(UniversalRobot::ConfigurationData)];
    UniversalRobot::ConfigurationData config;
  } configurationData;

  union {
    char raw[sizeof(UniversalRobot::ForceModeData)];
    UniversalRobot::ForceModeData fd;
  } forceModeData;

  union {
    char raw[sizeof(UniversalRobot::AdditionalInfo)];
    UniversalRobot::AdditionalInfo info;
  } additionalInfo;

  /*!
   * \brief Object for doing robot kinematics calculations
   * \see URDHKinematics
   * Contains information/routines necessary to compute
   * forward, inverse kinematics & jacobians.
   */
  URDHKinematics ur_kin_;

  /*!
   * \brief Current joint state
   * Raw joint state, as read from incoming UR packets.
   */
  URDHKinematics::JointVector cur_joints_;

  /*!
   * \brief Current joint velocities
   * Raw joint velocity, as read from incoming UR packets.
   */
  URDHKinematics::JointVelocityVector cur_joint_velocity_;
  URDHKinematics::JacobianMatrix cur_geo_jac_;

  /*!
   * \brief boolean to indicate if the robot is moving
   * This is updated each loop based on the robot speeds
   */
  bool moving;

  double a[6]; //!< DH a parameters, units in mm
  double d[6]; //!< DH d parameters, units in mm
  double alpha[6]; //!< DH alpha parameters, units in rad

  RobotInterface();
  ~RobotInterface();

  /*!
   * \brief Get joint state positions
   * Returns the raw joint state, as read from incoming UR packets.
   */
  void getPositions(double pos[6]);

  /*!
   * \brief Get the tool tip pose [x,y,z,rx,ry,rz]. The rotation is in axis/angle
   */
  void getToolPose(double pose[6]);

  struct UniversalRobot::RobotState getState() const;

  inline URDHKinematics::JacobianMatrix getGeoJacobian() const
  {
    return cur_geo_jac_;
  }

  inline URDHKinematics::JointVector cur_joints() const
  {
    return cur_joints_;
  }

  inline URDHKinematics::JointVelocityVector cur_joint_velocity() const
  {
    return cur_joint_velocity_;
  }

  inline bool isMoving() const
  {
    return moving;
  }

  /*!
   * \brief Parse the UR packets
   * \param buf The raw buffer
   * \param len The length of buf
   * Based on the parsed packet type, copy the bytes from the raw buffer
   * into the corresponding raw field of the union. Then, set useful class 
   * member variables using unionValue function.
   */
  void readPacket(const char *buf, const int len);

  /*!
   * \brief Read a packet from the socket buffer
   * \param CISST web socket abstraction object
   * \see readPacket
   * For reading packets sent from the UR robot.
   */
  void readFromSocket(osaSocket& sock);

public:
  /*!
   * \brief Save the DH parameters
   */
  void saveDH();

  // Create a DH matrix;
  //vctFrm4x4 DHMatrix(const double theta, const double d, const double a, const double alpha);

  /*!
   * \brief Switch the endianness of the union data
   * \todo Get rid of type-punning operations (bad practice!)
   * This function converts raw buffer endianness to machine endianness,
   * but does it by type punning, which could result in undefined behavior
   * on different platforms.
   */
  static int unionValue(UniversalRobot::intUnion u);

  /*!
   * \brief Switch the endianness of the union data
   * \todo Get rid of type-punning operations (bad practice!)
   * This function converts raw buffer endianness to machine endianness,
   * but does it by type punning, which could result in undefined behavior
   * on different platforms.
   */
  static float unionValue(UniversalRobot::floatUnion u);

  /*!
   * \brief Switch the endianness of the union data
   * \todo Get rid of type-punning operations (bad practice!)
   * This function converts raw buffer endianness to machine endianness,
   * but does it by type punning, which could result in undefined behavior
   * on different platforms.
   */
  static double unionValue(UniversalRobot::doubleUnion u);

  /*!
   * \brief Buffer for storing incoming UR packets
   */
  char ur_packet_buf[4096];

protected:
  /// Event for when the movement starts or stops.
  /// This passes a boolean along with the event to indicate if movement is started or stopped.
  mtsFunctionWrite MovementChangedTrigger;

};

//ETX

#endif // ROBOT_INTERFACE_H

