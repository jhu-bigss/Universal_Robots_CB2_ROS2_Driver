#ifndef MTSSAROBSYSTASK_H
#define MTSSAROBSYSTASK_H

// STD includes
#include <cstdlib>
#include <vector>

// cisst includes
#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnClassRegisterMacros.h>
#include <cisstVector.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmRobotState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <bigss_robotic_system/SeriallyAttachedRoboticSystem.h>
#include <bigss_robotic_system/SeriallyAttachedRoboticSystemHelper.h>
#include <bigss_constraint_controller/mtsBIGSSVFConfig.h>


#include <bigss_constraint_controller/mtsBIGSSVFController.h>
#include <queue>


class mtsSARobSysTask : public mtsTaskPeriodic
{
   CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)

  typedef std::queue<vct6> queueType;

  public:

  static const int DISABLED; // this is a stable state: once it's put here, it stays here
  static const int IDLE; // these states are set by the task, indicate status of optimizer
  static const int RUNNING; // all of them imply that the controller is enabled, and will
  static const int INFEASIBLE;// move the robot when given a goal
  static const int FAULT; // when it doesn't have all the information it needs

  /* constructors */
  mtsSARobSysTask(const std::string &name = "mtsSARobSysTask", SeriallyAttachedRoboticSystem* system = new SeriallyAttachedRoboticSystem(), mtsBIGSSVFConfig* VF_config =  new mtsBIGSSVFConfig(), double period_s=50.0*cmn_ms);
  
  // mtsSARobSysTask(mtsSARobSysTask& c); //copy constructor

  virtual ~mtsSARobSysTask();

  /* run workflow functions */
  void Run(void);
  void UpdateGoal();
  bool CheckIfGoalMet();
  void UpdateController();
  bool RunOptimizer();
  void MoveRobot();
  void stopMovement(const std::string &message="");

  void setupMultiTask();
  void SetControllerEnabled(const mtsBool &b);
  void SelectConstraints(const vctIntVec &toggles);

  void ClearGoal();
  void PopGoal();
  void SetGoal(const vct6 &p);
  void AddGoals(const vctMat &m);
  void MoveGoal(const vct6 &p);

  /* config functions */
  void SetPercentFullSpeed(const mtsDouble &s);
  void SetPrintProblem(const mtsBool &p);

  void SetMarkerOffset(const vct3 &offset);
  void SetRCM(const vct3 &p);

  void SetTipFeedbackMode(const mtsInt &i);
  void SetBaseFeedbackMode(const mtsInt &i);
  void SetVelocityJoint(const vctDoubleVec &joint_vels);
  void SetPositionJoint(const vctDoubleVec &joint_pos);

  void startVisualization();

void SetLowerJointVelLimits(const vctDoubleVec &lower);
void SetUpperJointVelLimits(const vctDoubleVec &upper);
void SetLowerAbsoluteJointLimits(const vctDoubleVec &lower);
void SetUpperAbsoluteJointLimits(const vctDoubleVec &upper);
void SetAxis(const vct3 &a);
void SetGains(const vct3 &g);
void SetJacobianWeights(const vctMat &w);
void SetGamma(const mtsDouble &g);

protected:
  
  SeriallyAttachedRoboticSystem* system;
  mtsBIGSSVFConfig* VF_config;
  int STATE_SIZE;
  mtsBIGSSVFController VF_controller;
  prmRobotState robotState;
  vctDoubleVec JointVelocityCommand;
  queueType goalpoints;

  vct3 rcm;
  vct3 markerOffset;
  vct3 polarisTip;
  vct3 fbgTip;
  prmPositionCartesianGet URBaseToToolMarker;
  vctFrm3 urBaseToPolarisBaseMarker;
  vctFrm3 polarisToolToSnakeBase;
  vctFrm3 fkTooltip;

  prmPositionCartesianGet snakeTipGroundTruth;
  mtsFunctionRead GetPolarisTip;
  mtsFunctionRead GetFbgTip;

  mtsFunctionRead GetPolarisBase;
  mtsFunctionRead GetPolarisTool;
  mtsFunctionRead get_measured_jp;
  
  mtsFunctionRead GetGroundTruth;
  mtsFunctionRead GetPolarisBaseMarker;
  mtsFunctionRead GetPolarisToolMarker;

  prmStateJoint JointStates;  
  
  /* control variables */
  double goalThresh;
  double percentFullSpeed;
  int status;
  bool printProblem;

  bool sim;
  bool interpolate_goal_at_go;
  vct6 next_interp_goal;
  vctDoubleVec next_interp_goal_vctDoubleVec;

  vct3 markerPos;
  prmPositionCartesianGet polarisBase;
  prmPositionCartesianGet polarisTool;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSARobSysTask);

#endif // MTSSAROBSYSTASK_H
