/*! \file robotInterface.cpp */

#include "ur_cb2_robot_driver/ur/RobotInterface.h"

#include <cstring>
#include <cisstOSAbstraction/osaSocket.h>

RobotInterface::RobotInterface() :
  moving(false)
{
  memset(&jointData.raw[0], 0, sizeof(jointData));
  memset(&robotMode.raw[0], 0, sizeof(robotMode));
  memset(&toolData.raw[0], 0, sizeof(toolData));
  memset(&masterboardData.raw[0], 0, sizeof(masterboardData));
  memset(&masterboardDataWithEuromap.raw[0], 0, sizeof(masterboardDataWithEuromap));
  memset(&cartesianInfo.raw[0], 0, sizeof(cartesianInfo));
  memset(&configurationData.raw[0], 0, sizeof(configurationData));
  memset(&forceModeData.raw[0], 0, sizeof(forceModeData));
  memset(&additionalInfo.raw[0], 0, sizeof(additionalInfo));
}

RobotInterface::~RobotInterface()
{
}

struct UniversalRobot::RobotState RobotInterface::getState() const
{
  return robotMode.rmd.state;
}

void RobotInterface::readPacket(const char *buf, const int len)
{

  if(buf[0] != char(16)) {
    std::cerr << "Unexpected packet type" << std::endl;
    return;
  }
  buf++;
  int readBytes = 1;
  int bytesToRead = 0;
  UniversalRobot::intUnion packLen;
  int packType;

  // read in each individual packet
  while(readBytes < len) {
    memcpy(&packLen.raw[0], &buf[0], 4);
    packType = buf[4];
    bytesToRead = unionValue(packLen);

    switch(packType) {
    case UniversalRobot::ROBOT_MODE_DATA:
      memcpy(&robotMode.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::JOINT_DATA:
      memcpy(&jointData.raw[0], &buf[0], bytesToRead);

      // compute the Jacobian
      for(int i=0; i<6; i++)
      {
        cur_joint_velocity_[i] = unionValue(jointData.jd.joint[i].qd_actual);
        cur_joints_[i] = unionValue(jointData.jd.joint[i].q_actual);
      }
      cur_geo_jac_ = ur_kin_.geo_jac(cur_joints_);

      break;
    case UniversalRobot::TOOL_DATA:
      memcpy(&toolData.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::MASTERBOARD_DATA:
      if(bytesToRead == 72)
        memcpy(&masterboardData.raw[0], &buf[0], bytesToRead);
      else
        memcpy(&masterboardDataWithEuromap.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::CARTESIAN_INFO:
      memcpy(&cartesianInfo.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::KINEMATICS_INFO:
      // nothing to do here
      break;
    case UniversalRobot::CONFIGURATION_DATA:
      memcpy(&configurationData.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::FORCE_MODE_DATA:
      memcpy(&forceModeData.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::ADDITIONAL_INFO:
      memcpy(&additionalInfo.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::CALIBRATION_DATA:
      // nothing to do here
      break;
    default:
      std::cerr << "Unrecognized packet " << packType << std::endl;
      return;
    }

    readBytes += bytesToRead;
    buf += bytesToRead;
  }
}

void RobotInterface::saveDH()
{
  for(int i=0; i<6; i++) {
    a[i] = unionValue(configurationData.config.dh_a[i]) * 1000.0;
    d[i] = unionValue(configurationData.config.dh_d[i]) * 1000.0;
    alpha[i] = unionValue(configurationData.config.dh_alpha[i]);
  }
}

// This will eventually replace the run method
void RobotInterface::readFromSocket(osaSocket& sock)
{
  UniversalRobot::intUnion len;
  len.data = 0;
  int nBytes = sock.Receive(&len.raw[0], 4);
  if((nBytes != 4) && (nBytes != 0)) {
    // bad packet
    std::cerr << "Bad packet length" << std::endl;
    return;
  }

  int packetLength = unionValue(len);
  if((packetLength <= 4) || (packetLength > 4096))
    return;

  // read the packet, less the 4 bytes for length
  int packetBytes = packetLength - 4;
  nBytes = sock.Receive(&ur_packet_buf[0], packetBytes);
  if(nBytes != packetBytes) {
    // bad packet
    std::cerr << "Bad packet" << std::endl;
    return;
  }
  //std::cout << "read bytes: " << packetBytes + 4 << std::endl;
  readPacket(ur_packet_buf, packetBytes);
}

void RobotInterface::getPositions(double pos[6])
{
  for(int i=0; i<6; i++)
    pos[i] = unionValue(jointData.jd.joint[i].q_actual);
}

void RobotInterface::getToolPose(double pose[6])
{
  pose[0] = unionValue(cartesianInfo.info.x);
  pose[1] = unionValue(cartesianInfo.info.y);
  pose[2] = unionValue(cartesianInfo.info.z);
  pose[3] = unionValue(cartesianInfo.info.Rx);
  pose[4] = unionValue(cartesianInfo.info.Ry);
  pose[5] = unionValue(cartesianInfo.info.Rz);
}

int RobotInterface::unionValue(UniversalRobot::intUnion u)
{
  UniversalRobot::intUnion tmp;
  for(int i=0; i<4; i++)
    tmp.raw[3-i] = u.raw[i];

  return tmp.data;
}

float RobotInterface::unionValue(UniversalRobot::floatUnion u)
{
  UniversalRobot::floatUnion tmp;
  for(int i=0; i<4; i++)
    tmp.raw[3-i] = u.raw[i];

  return tmp.data;
}

double RobotInterface::unionValue(UniversalRobot::doubleUnion u)
{
  UniversalRobot::doubleUnion tmp;
  for(int i=0; i<8; i++)
    tmp.raw[7-i] = u.raw[i];

  return tmp.data;
}

