#include "ur_robot_driver/ur/RobotServer.h"

#include <cstdlib>

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmRobotState.h>
#include <cisstVector.h>

#include <ur_robot_driver/ur/bigss_debug_util.h>

#include "ur_robot_driver/ur/URDHKinematics.h"

CMN_IMPLEMENT_SERVICES_DERIVED(RobotServer, mtsTaskPeriodic)

namespace
{

// Returns the localhost IP, but first checks to see if an environment variable
// is declared to specify an IP to use. This is useful when the first IP returned
// by osaSocket::GetLocalhostIP is not accessible to the UR 5 (as is the case
// when using the simulator in the VM).
std::string GetLocalhostIP()
{
  std::string ip_str;

  char* env_val = getenv("BIGSS_PC_IP");

  if (env_val)
  {
    ip_str = env_val;
  }
  else
  {
    ip_str = osaSocket::GetLocalhostIP();
  }

  // return ip_str;
  return "192.168.2.3";
}

// Replace all instances of key in a src_str with a val string and return the
// new string
std::string ReplaceAll(const std::string& src_str, const std::string& key,
                       const std::string& val)
{
  std::string new_str = src_str;

  const size_t key_len = key.size();

  size_t key_pos = new_str.find(key);

  while (key_pos != new_str.npos)
  {
    new_str = new_str.replace(key_pos, key_len, val);

    key_pos = new_str.find(key);
  }

  return new_str;
}

}  // un-named

RobotServer::RobotServer(const std::string &name, const unsigned short &sp) :
  mtsTaskPeriodic(name, 10*cmn_ms),
  progSocket(0),
  isConnectedToProg(false),
  host_port(sp),
  isConnectedToUR(false),
  trajTolerance (0.01),
  trajID (0),
  trajVel (0.02),
  commandingTraj (false),
  ur_kins_(),
  use_high_level_pd_(false),
  ur_interface(),
  JointPos(6),
  CartPos(),
  JointVel(6),
  CartVel(),
  velocityTimeLimit(true),
  lastVelocityCommand(),
  lastVelocityCommandThresh(1000),
  lastVelocityCommandTime(bigss::time_now_ms())
{
  this->StateTable.AddData(this->JointPos, "JointPos");
  this->StateTable.AddData(this->CartPos, "CartPos");
  this->StateTable.AddData(this->JointVel, "JointVel");
  this->StateTable.AddData(this->CartVel, "CartVel");
  this->StateTable.AddData(this->GeoJacobian, "GeoJacobian");

  mtsInterfaceProvided *provided = AddInterfaceProvided("server");
  if(provided) {
    provided->AddCommandWrite(&RobotServer::SendToURClient, this, "SendToURClient");
    provided->AddCommandWrite(&RobotServer::SetPositionJoint, this, "SetPositionJoint");
    provided->AddCommandWrite(&RobotServer::SetVelocityJoint, this, "SetVelocityJoint");
    provided->AddCommandWrite(&RobotServer::SetPositionCartesian, this, "SetPositionCartesian");
    provided->AddCommandWrite(&RobotServer::SetVelocityCartesian, this, "SetVelocityCartesian");
    provided->AddCommandWrite(&RobotServer::SetUseHighLevelPDControl, this, "SetUseHighLevelPDControl");
    provided->AddCommandWrite(&RobotServer::SetPDSpeedPercent, this, "SetPDSpeedPercent");
    provided->AddCommandWrite(&RobotServer::SetSpeedPercent, this, "SetSpeedPercent");
    provided->AddCommandVoid(&RobotServer::StopMotion, this, "Stop");
    provided->AddCommandReadState(this->StateTable, JointPos, "GetPositionJoint");
    provided->AddCommandReadState(this->StateTable, CartPos, "GetPositionCartesian");
    provided->AddCommandReadState(this->StateTable, JointVel, "GetVelocityJoint"); 
    provided->AddCommandReadState(this->StateTable, CartVel, "GetVelocityCartesian"); 
    provided->AddCommandReadState(this->StateTable, GeoJacobian, "GetGeoJacobian");
  }
}

RobotServer::~RobotServer()
{
  if(progSocket != 0) {
    delete progSocket; // close the server socket
    progSocket = 0;
  }

  server.Close();

  if (isConnectedToUR)
  {
    urSocket.Close();
  }
}

void RobotServer::SetupURProgSocket()
{
  if (server.AssignPort(host_port))
  {
    if (!server.Listen())
    {
      CMN_LOG_CLASS_INIT_DEBUG << "warning, failed to listen on server port!" << std::endl;
    }
    else
    {
      CMN_LOG_CLASS_INIT_DEBUG << "listening on " << host_port << std::endl;
    }
  }
  else
  {
    CMN_LOG_CLASS_INIT_DEBUG << "warning, failed to assign server port!" << std::endl;
  }
}

void RobotServer::AssignServerPort(const unsigned short port)
{
  this->host_port = port;
}

void RobotServer::Startup()
{
}

void RobotServer::Run()
{
  ProcessQueuedCommands();
  ProcessQueuedEvents();

  // Parse status from the UR
  if (isConnectedToUR)
  {
    // update the state using the default state updates the UR gives over
    // the specified UR socket
    ur_interface.readFromSocket(urSocket);

    //ur_interface.saveDH();
    //for(int i=0; i<6; i++) {
    //  std::cout << "a: " << ur_interface.a[i];
    //  std::cout << " alpha: " << ur_interface.alpha[i];
    //  std::cout << " d: " << ur_interface.d[i] << std::endl;
    //}

    this->JointPos.Position() = vctDoubleVec(ur_interface.cur_joints());
    this->UpdatePositionCartesian(this->CartPos.Position());
    this->JointVel.Velocity() = vctDoubleVec(ur_interface.cur_joint_velocity());
    this->GeoJacobian = ur_interface.getGeoJacobian();
    // this->UpdateVelocityCartesian(this->CartVel.Velocity());

    if (velocityTimeLimit)
    {
      if(lastVelocityCommand.Norm() > 0.0001
       && bigss::time_now_ms() - lastVelocityCommandTime > lastVelocityCommandThresh)
      {
        StopMotion();
        lastVelocityCommand.SetAll(0.0);
      }
    }

    if (commandingTraj)
    {
      CMN_LOG_CLASS_RUN_WARNING << "commanding traj" << std::endl;

      if (CheckTrajUpdate())
      {
        std::cout << "updating traj id" << std::endl;
        trajID ++;
      }

      // if reached end of trajectory, stop the motion
      if (trajID >= jointTraj.cols())
      {
        commandingTraj = false;
        StopMotion();
      }
      else
        StepTraj ();
    }
  }
}

void RobotServer::SendToURClient(const mtsStdString &str)
{
  if(progSocket == 0) {
    CMN_LOG_CLASS_RUN_WARNING << "No valid server socket" << std::endl;
    return;
  }

  CMN_LOG_CLASS_RUN_DEBUG << "Sending message: " << str.Data.c_str() << std::endl;

#ifdef OSA_SOCKET_WITH_STREAM
  *progSocket << str.c_str();
#else
  progSocket->Send(str.Data.c_str());
#endif // OSA_SOCKET_WITH_STREAM
}

void RobotServer::SetPositionJoint(const vctDoubleVec& joints)
{
  if (use_high_level_pd_) // this is by default false
  {
    SendDoubleVec(PD_JOINT_SPACE, joints);
  }
  else
  {
    SendDoubleVec(JOINT_POSITION, joints);
  }
}

void RobotServer::SetVelocityJoint(const vctDoubleVec& joints_vels)
{
  CMN_LOG_CLASS_RUN_DEBUG << "Sending joint velocity: " << joints_vels << std::endl;
  lastVelocityCommandTime = bigss::time_now_ms();
  lastVelocityCommand = joints_vels;
  SendDoubleVec(JOINT_VELOCITY, joints_vels);
}

void RobotServer::SetPositionCartesian(const vctFrm3& pose)
{
  vctDoubleVec ur_pose(6);

  std::cout << "cart pose command = " << std::endl << pose << std::endl;

  ConvertFrm3ToURPose(pose, ur_pose);

/*  if (use_high_level_pd_)
  {
    SendDoubleVec(PD_CARTESIAN, ur_pose);
  }
  else
  {
  */
    SendDoubleVec(SPATIAL_POSITION, ur_pose);
  //}
}

void RobotServer::UpdatePositionCartesian(vctFrm3& pose) const
{
  pose.Translation()[0] = RobotInterface::unionValue(ur_interface.cartesianInfo.info.x);
  pose.Translation()[1] = RobotInterface::unionValue(ur_interface.cartesianInfo.info.y);
  pose.Translation()[2] = RobotInterface::unionValue(ur_interface.cartesianInfo.info.z);

  vct3 axis(RobotInterface::unionValue(ur_interface.cartesianInfo.info.Rx),
            RobotInterface::unionValue(ur_interface.cartesianInfo.info.Ry),
            RobotInterface::unionValue(ur_interface.cartesianInfo.info.Rz));

  vctAxAnRot3 axis_ang_rot;
  axis_ang_rot.Angle() = axis.Norm();
  if (axis_ang_rot.Angle() != 0)
    axis_ang_rot.Axis() = axis.Normalized();

  pose.Rotation().From(axis_ang_rot);
}

void RobotServer::SetVelocityCartesian(const vctDoubleVec& vel)
{
  CMN_LOG_CLASS_RUN_DEBUG << "received velocity " << vel << std::endl;
  SendDoubleVec(SPATIAL_VELOCITY, vel);
}

void RobotServer::UpdateVelocityCartesian(vctDoubleVec& vel) const
{
  // convert joint velocity to cartesian velocity with the jacobian
  // e.g. J q_dot = x_dot

  // joint angles
  vctDoubleVec q(6);
  q = vctDoubleVec(ur_interface.cur_joints());

  // joint angle velocity
  vctDoubleVec q_dot = vctDoubleVec(ur_interface.cur_joint_velocity());
  vel = vctDoubleVec(ur_kins_.geo_jac(q) * vct6(q_dot));
}

void RobotServer::SendDoubleVec(const unsigned long mode, const vctDoubleVec& v, const bool pad)
{
  typedef vctDoubleVec::size_type size_type;

  std::ostringstream oss;
  oss << '(' << mode;

  const size_type v_len = v.size();
  for (size_type i = 0; i < v_len; ++i)
  {
    oss << ',' << v[i];
  }

  // TODO: have this 6 as a constant somewhere
  // pad out to the maximum length
  if(pad) {
    const size_type pad_len = 6 - v_len;
    for (size_type i = 0; i < pad_len; ++i)
    {
      oss << ",0";
    }
  }

  oss << ')';

  SendToURClient(oss.str());
}

bool RobotServer::ConnectToUR(const std::string& ur_host, unsigned short ur_port)
{
  isConnectedToUR = urSocket.Connect(ur_host.c_str(), ur_port);
  CMN_LOG_CLASS_INIT_DEBUG << "Is connected to UR? = " << isConnectedToUR << std::endl;
  return isConnectedToUR;
}

void RobotServer::ProgramFromFile(const std::string &filename)
{

  std::ifstream file(filename.c_str(), std::ifstream::in);
  if(!file) {
    std::cerr << "Invalid program file: " << filename << std::endl;
    return;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  SendProgramToUR(buffer.str());
}

void RobotServer::SendProgramToUR(const std::string &program, bool acceptSocket)
{
  // first close socket to previous program, if established
  if (isConnectedToProg)
  {
    progSocket->Close();
    delete progSocket;
    progSocket = 0;
    isConnectedToProg = false;
  }
  else
  {
    SetupURProgSocket();
  }

  // set ip address of the host computer for the robot program
  std::string new_str = ReplaceAll(program, "${HOSTNAME}",
                                   GetLocalhostIP());

  // set the port of the host RobotServer process for the robot program
  std::stringstream portStream;
  portStream << host_port;
  new_str = ReplaceAll(new_str, "${HOSTPORT}", portStream.str());

  // there must be an additional new line to start the program
  new_str.push_back('\n');
  CMN_LOG_CLASS_RUN_DEBUG << "Sending UR Program: " << std::endl
    << new_str << std::endl;
  urSocket.Send(new_str.c_str(), (unsigned int)new_str.length());

  // if we don't care about communicating with the program, return
  if(!acceptSocket)
    return;

  // for some reason, this loop is now necessary on Windows systems?
  int count = 0;
  do {
    progSocket = server.Accept();
    osaSleep(50.0*cmn_ms);
    CMN_LOG_CLASS_INIT_DEBUG << "waiting for connection" << std::endl;
    // count ++;
  } while (progSocket == 0 && count < 20);

  isConnectedToProg = (progSocket != 0);
  CMN_LOG_CLASS_INIT_DEBUG << "UR Connection established? = "
    << isConnectedToProg << std::endl;
}

void RobotServer::GetJointModes(vctInt6 &modes) const
{
  for (unsigned long i = 0; i < 6; ++i)
  {
    modes[i] = ur_interface.jointData.jd.joint[i].jointMode;
  }
}

void RobotServer::IsJointHomed(vctBool6 &stateVec) const
{
  // TODO: Handle UR 3.0 case
  for (unsigned long i = 0; i < 6; ++i)
  {
    stateVec[i] = ur_interface.jointData.jd.joint[i].jointMode == UniversalRobot::JOINT_INITIALIZATION;
  }
}

void RobotServer::IsHomed(bool &state) const
{
  bool homed = true;

  // TODO: Handle UR 3.0 case
  for (unsigned long i = 0; homed && (i < 6); ++i)
  {
    homed = homed && (ur_interface.jointData.jd.joint[i].jointMode == UniversalRobot::JOINT_INITIALIZATION);
  }

  state = homed;
}

void RobotServer::IsMotorPowerOn(bool &state) const
{
  state = ur_interface.robotMode.rmd.state.isPowerOnRobot;
}

void RobotServer::IsEStop(bool &state) const
{
  state = ur_interface.robotMode.rmd.state.isEmergencyStopped;
}

// Return true if security stop activated (software-controlled stop)
void RobotServer::IsSecurityStop(bool &state) const
{
  state = ur_interface.robotMode.rmd.state.isProtectedStopped;
}

void RobotServer::SetRobotFreeDriveMode(void)
{
  SendProgramToUR("stop program");
  SendProgramToUR("set robotmode freedrive");
}

void RobotServer::SetRobotRunningMode(void)
{
  SendProgramToUR("set robotmode run");
}

void RobotServer::StopMotion(void)
{
  // stop commanding a trajectory
  CMN_LOG_CLASS_RUN_DEBUG << "Stopping robot" << std::endl;
  commandingTraj = false;

  vctDoubleVec joint_acc(1);
  joint_acc[0] = 3;
  SendDoubleVec(STOP_JOINT_SPACE, joint_acc);
}

void RobotServer::FwdKin(const vctDoubleVec& joints, vctFrm3& pose, vctDoubleMat& jac)
{
  pose.From(vctFrm4x4(ur_kins_.pose(joints)));
  jac  = ur_kins_.geo_jac(joints);
}

void RobotServer::InvKin(const vctFrm3& pose, vctDoubleVec& joints, const vctDoubleVec &current_joints)
{
  vctDoubleVec ur_pose(6);
  ConvertFrm3ToURPose(pose, ur_pose);

  joints.resize(6);

  vctFrm4x4 pose4x4(pose);
  vctDoubleMat q;
  q.SetSize(6, 8);

  int nSols = ur_kins_.inverse_kinematics(pose4x4, q);

  if(nSols > 0) {

    // Use this to grab the pose from the robot
    vctDoubleVec current_joints = vctDoubleVec(ur_interface.cur_joints());

    ur_kins_.find_closest_ik(q, current_joints, joints);
  }
}

void RobotServer::ConvertFrm3ToURPose(const vctFrm3 vct_pose, vctDoubleVec& ur_pose)
{
  // The UR needs a pose in axis angle format; the first 3 elements are the
  // translation component and the second 3 represent the rotation axis scaled
  // by the rotation angle

  vctAxAnRot3 axis_ang_rot(vct_pose.Rotation());
  vctFixedSizeVectorRef<double,3,1> ur_ax_ang(&ur_pose[3]);
  ur_ax_ang  = axis_ang_rot.Axis();
  ur_ax_ang *= axis_ang_rot.Angle()*180/M_PI;

  vctFixedSizeVectorRef<double,3,1> trans(&ur_pose[0]);
  trans = vct_pose.Translation();

  std::cout << "ur pose was computed to be:" << std::endl << ur_pose << std::endl;
}

void RobotServer::SetUseHighLevelPDControl(const bool& b)
{
  use_high_level_pd_ = b;
}

void RobotServer::SetPDSpeedPercent(const double& value)
{
  vctDoubleVec v(1);
  v.SetAll(value/100.0);
  SendDoubleVec(SET_PD_SPEED, v, false);
}

void RobotServer::SetSpeedPercent(const mtsInt &value)
{
  vctDoubleVec v(1);
  v.SetAll(value/100.0);
  CMN_LOG_CLASS_RUN_WARNING << "setting speed percent to %" << value << std::endl;
  SendDoubleVec(SET_SPEED_PERCENT, v, false);
}

void RobotServer::GetUseHighLevelPDControl(bool& b)
{
  b = use_high_level_pd_;
}

void RobotServer::SetURType(const bool isUR5)
{
  if(isUR5)
    ur_kins_.set_ur5();
  else
    ur_kins_.set_ur10();
}

void RobotServer::ReadDoubles(const unsigned int n, vctDoubleVec &doubles)
{
  if(progSocket == 0) {
    std::cout << "No valid server socket" << std::endl;
    return;
  }

  char buffer[256];
  for (unsigned int i=0; i<n; ) {
    int bytesRead = progSocket->Receive(buffer, sizeof(buffer), 20.0 * cmn_ms);
    if(bytesRead == 0) {
      std::cout << "No text read." << std::endl;
      return;
    }
    buffer[bytesRead] = '\0';
    std::stringstream ss;
    ss << buffer;
    double num;
    while(ss >> num) {
      doubles[i] = num;
      i ++;
    }

  }
}

void RobotServer::CommandCartesianTrajectory(const std::vector <vctFrm4x4> &poses, const double vel)
{
  trajVel = vel;

  vctDoubleVec prevJoint(6);
  vctDoubleVec nextJoint(6);
  vctDoubleVec jointVel(6);

  // Stop joint space motion
  StopMotion();

  // get the current position
  prevJoint = vctDoubleVec(ur_interface.cur_joints());

  trajID = 0;

  vctDoubleMat jointTrajTemp;
  jointTrajTemp.SetSize (6, poses.size());
  int nPoses = 0;
  for (int i=0; i<poses.size(); i++)
  {
    // get the inverse kinematics closest to the previous point
    InvKin(vctFrm3(poses[i]), nextJoint, prevJoint);

    std::cout << "next " << nextJoint << std::endl;

    if ((nextJoint - prevJoint).Norm() >= 2*trajTolerance)
    {
      std::cout << "storing traj" << std::endl;
      // store the trajectory
      jointTrajTemp.Column(nPoses).Assign (nextJoint);

      prevJoint = nextJoint;
      nPoses ++;
    }
  }

  // resize this appropriately 
  // (apparently, the resize function IS a desctructive size change)
  jointTraj.SetSize(6, nPoses);
  jointTraj.Assign(jointTrajTemp.Ref(6, nPoses));

  std::cout << "command traj step 1" << std::endl;
  std::cout << "nPoses : " << nPoses << std::endl;
  std::cout << "jointTraj: " << jointTraj << std::endl;

  if (nPoses == 0)
    std::cout << "No valid trajectory data provided." << std::endl;

  commandingTraj = true;

  // compute the velocity
  prevJoint = vctDoubleVec(ur_interface.cur_joints());
  jointVel = (jointTraj.Column(0) - prevJoint).Normalized() * vel;
  std::cout << "jointVel " << jointVel << std::endl;
  SetVelocityJoint (jointVel);
} 

//---------------------------------------------------------------------------
bool RobotServer::CheckTrajUpdate ()
{
  vctDoubleVec currentJoints(6);

  currentJoints = vctDoubleVec(ur_interface.cur_joints());
  vctDoubleVec diff = jointTraj.Column(trajID) - currentJoints;

  bool complete = diff.Norm() < trajTolerance;
  return complete;
}

//---------------------------------------------------------------------------
void RobotServer::StepTraj ()
{
  vctDoubleVec currentJoints(6);
  vctDoubleVec vel(6);

  currentJoints = vctDoubleVec(ur_interface.cur_joints());
  vel = (jointTraj.Column(trajID) - currentJoints).Normalized() * trajVel;

  SetVelocityJoint (vel);
}

void RobotServer::GetRobotState(UniversalRobot::RobotState &s) const
{
  s = ur_interface.getState();
}

