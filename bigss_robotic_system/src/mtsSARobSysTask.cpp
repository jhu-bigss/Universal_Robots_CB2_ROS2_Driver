#include <bigss_robotic_system/mtsSARobSysTask.h>
#include <bigss_robotic_system/SeriallyAttachedRoboticSystem.h>
#include <bigss_constraint_controller/mtsBIGSSVFConfig.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstMultiTask/mtsGenericObjectProxy.h>

#include <cisstParameterTypes/prmRobotState.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstCommon/cmnUnits.h>
#include <chrono>
#include <iostream>
#include <fstream>

//----------------------------------------------------------------------------
// constructors

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSARobSysTask, mtsTaskPeriodic);
const int mtsSARobSysTask::DISABLED = 0;
const int mtsSARobSysTask::IDLE = 1;
const int mtsSARobSysTask::RUNNING = 2;
const int mtsSARobSysTask::INFEASIBLE = 3;

mtsSARobSysTask::mtsSARobSysTask(const std::string &name, SeriallyAttachedRoboticSystem* system, mtsBIGSSVFConfig* VF_config, double period_s):
  mtsTaskPeriodic(name, period_s),
  system(system),
  VF_config(VF_config),
  STATE_SIZE(system->GetNumberOfJoints()),
  VF_controller(STATE_SIZE), //TODO: try mode=mtsVFBase::JVEL
  goalpoints(),
  goalThresh(0.5),
  percentFullSpeed(1.0),
  status(DISABLED),//TODO: should something externally really have to send this the first time? I think idea was to wait for GUI to be done configuring
  printProblem(false),
  sim(sim)
{
  rcm(0) = 0.0; rcm(1) = 0.0; rcm(2) = 0.0; //TODO: get rid of this init
  setupMultiTask();
}

mtsSARobSysTask::~mtsSARobSysTask(){}

void mtsSARobSysTask::Run(void)
{
  bool started_loop_disabled = (status == DISABLED);

  ProcessQueuedCommands();
  ProcessQueuedEvents();

  if (!system->UpdateState())
      {status = DISABLED;}
  
  if (status == DISABLED){
    if (!started_loop_disabled)
      {system->StopMovement();}
    return;
  }
  status = IDLE;  
  UpdateGoal();
  if (goalpoints.empty()) 
    {system->StopMovement(); return;}
  if (CheckIfGoalMet())
    {system->StopMovement(); return;}
  UpdateController();
  if (!RunOptimizer()) 
    {system->StopMovement(); return;}
  MoveRobot();
}

void mtsSARobSysTask::UpdateGoal()
{
  system->system_state.CartesianPositionGoal() = !goalpoints.empty() ?
    vctVec(goalpoints.front()) : system->system_state.CartesianPosition();
  //TODO:Calculate error here
 
  //TODO: change how VF works so you don't have to do this goal state
  ConvertR6ToFrm3(system->system_state.CartesianPositionGoal(), system->goal_kinematics_state.Frame);
}

bool mtsSARobSysTask::CheckIfGoalMet(){
  double position_err_threshold = 0.5*cmn_mm;
  double rotation_err_threshold = 0.5*cmnPI_180;
  rotation_err_threshold = MAXFLOAT;//TEMP
  
  vct3 position_err, rotation_err; //TODO: check implementation of error
  CalculateCartesianErrorFromR6(system->system_state.CartesianPositionGoal(), system->system_state.CartesianPosition(), position_err, rotation_err);
  std::copy(position_err.begin(),position_err.end(),system->system_state.CartesianPositionError().begin());
  std::copy(rotation_err.begin(),rotation_err.end(),system->system_state.CartesianPositionError().begin()+3);

  if ( (position_err.Norm() < position_err_threshold) && (rotation_err.Norm() < rotation_err_threshold)){
    this->goalpoints.pop();
    CMN_LOG_CLASS_RUN_VERBOSE << "goal reached!" << std::endl;
    if (this->goalpoints.empty())
      {return true;}
  }
  return false;
}

void mtsSARobSysTask::UpdateController()
{
  VF_config->SetAllActiveVFs(VF_controller);
  VF_controller.UpdateOptimizer(StateTable.GetAveragePeriod()); //TODO: Is this better than using 0.05 i.e. dt?
}

bool mtsSARobSysTask::RunOptimizer()
{
  status = RUNNING;
  if (this->printProblem)
    {VF_controller.PrintProblem();}
  
  bool solved = VF_controller.Solve(system->system_state.JointVelocityGoal());
  if (!solved) 
    {status = INFEASIBLE;}
  if (status != RUNNING)
    {return false;}
  return true;
}

void mtsSARobSysTask::MoveRobot()
{
  vctDoubleVec& joint_vel_cmd = system->system_state.JointVelocityGoal();
  joint_vel_cmd *= percentFullSpeed;
  JointVelocityCommand = joint_vel_cmd;
  prmVelocityJointSet system_command;
  system_command.SetSize(system->GetNumberOfJoints());
  system_command.SetGoal(joint_vel_cmd);
  system->Move(system_command);
}
void 

mtsSARobSysTask::stopMovement(const std::string &message) //Bad implementation
{
  CMN_LOG_CLASS_RUN_VERBOSE << "stopping movement: " << message << std::endl;
  system->StopMovement();
}

void mtsSARobSysTask::setupMultiTask()
{
  // for(auto& required_interface_name: system->own_required_task_names){
  //   this->AddInterfaceRequired(required_interface_name);
  // }


  //TEMP ADD CRTK-compliant for crtk cisst-ros bridge TODO: Fix throughout
  mtsInterfaceProvided *crtk_provided = this->AddInterfaceProvided("crtk_interface");
  for( auto& robot : system->robot_interfaces)
  {
      this->StateTable.AddData(robot->m_measured_cp, robot->GetName()+ "_measured_cp" );
      crtk_provided->AddCommandReadState(StateTable, robot->m_measured_cp, robot->GetName()+ "/measured_cp");
      this->StateTable.AddData(*(robot->kinematics_state.JointState), robot->GetName()+"_measured_js");
      crtk_provided->AddCommandReadState(StateTable, *(robot->kinematics_state.JointState), robot->GetName()+ "/measured_js");
     
      this->StateTable.AddData(robot->m_base_offset_measured_cp, robot->GetName()+ "_base_offset_measured_cp" );
      crtk_provided->AddCommandReadState(StateTable, robot->m_base_offset_measured_cp, robot->GetName()+ "/base_offset/measured_cp");

  }
  this->StateTable.AddData(system->system_measured_cp, system->name +  "_measured_cp" );
  crtk_provided->AddCommandReadState(StateTable, system->system_measured_cp, system->name +  "/measured_cp");
  this->StateTable.AddData(*(system->kinematics_state.JointState), system->name+"_measured_js");
  crtk_provided->AddCommandReadState(StateTable, *(system->kinematics_state.JointState), system->name+ "/measured_js");




  this->StateTable.AddData(VF_config->VF_joint_vel_limits_data.LowerLimits, "LowerJointVelLimits");
  this->StateTable.AddData(VF_config->VF_joint_vel_limits_data.UpperLimits, "UpperJointVelLimits");
  this->StateTable.AddData(VF_config->VF_absolute_joint_limits_data.LowerLimits, "LowerAbsoluteJointLimits");
  this->StateTable.AddData(VF_config->VF_absolute_joint_limits_data.UpperLimits, "UpperAbsoluteJointLimits");
  this->StateTable.AddData(this->rcm, "RCM");
  this->StateTable.AddData(this->rcm, "Axis"); //TODO FIX THIS
  this->StateTable.AddData(system->system_state, "State");
  this->StateTable.AddData(this->status, "Status");
  this->StateTable.AddData(VF_config->VF_follow_data.gains, "Gains"); 
  this->StateTable.AddData(system->system_state.JointVelocityGoal(), "VelocityGoal");
  this->StateTable.AddData(this->JointVelocityCommand, "VelocityCommand");
  this->StateTable.AddData(this->next_interp_goal_vctDoubleVec, "InterpCartesianPositionGoal");
  this->StateTable.AddData(system->kinematics_state.JointState->Position(), "Joints");
  this->StateTable.AddData(system->system_state.CartesianPosition(), "CartesianPosition"); //Need a way to access the kins w/o sensor
  this->StateTable.AddData(system->system_state.CartesianPositionGoal(), "CartesianPositionGoal");
  this->StateTable.AddData(fkTooltip, "FKTooltip"); //TODO::TEMP SHOULD REMOVE
  this->StateTable.AddData(VF_config->VF_follow_data.weights, "JacobianWeights");
  this->StateTable.AddData(system->kinematics_state.Jacobian, "Jacobian");
  this->StateTable.AddData(system->system_measured_cp, "system_measured_cp");

  // setup the user interface
  mtsInterfaceProvided *provided = this->AddInterfaceProvided("userInterface");
  provided->AddCommandWrite(&mtsSARobSysTask::SetLowerJointVelLimits, this, "SetLowerJointVelLimits");
  provided->AddCommandWrite(&mtsSARobSysTask::SetUpperJointVelLimits, this, "SetUpperJointVelLimits");
  provided->AddCommandWrite(&mtsSARobSysTask::SetLowerAbsoluteJointLimits, this, "SetLowerAbsoluteJointLimits");
  provided->AddCommandWrite(&mtsSARobSysTask::SetUpperAbsoluteJointLimits, this, "SetUpperAbsoluteJointLimits");
  provided->AddCommandWrite(&mtsSARobSysTask::SetAxis, this, "SetAxis");
  provided->AddCommandWrite(&mtsSARobSysTask::SetGains, this, "SetGains");
  provided->AddCommandWrite(&mtsSARobSysTask::SetJacobianWeights, this, "SetJacobianWeights");
  provided->AddCommandWrite(&mtsSARobSysTask::SetGamma, this, "SetGamma");
  // provided->AddCommandWrite(&mtsSARobSysTask::SetSnakeJacobian, this, "SetSnakeJacobian");
  provided->AddCommandWrite(&mtsSARobSysTask::SelectConstraints, this, "SelectConstraints");
  provided->AddCommandWrite(&mtsSARobSysTask::SetControllerEnabled, this, "SetControllerEnabled");
  provided->AddCommandWrite(&mtsSARobSysTask::SetMarkerOffset, this, "SetMarkerOffset");
  provided->AddCommandWrite(&mtsSARobSysTask::SetPercentFullSpeed, this, "SetPercentFullSpeed");
  provided->AddCommandWrite(&mtsSARobSysTask::SetPrintProblem, this, "SetPrintProblem", mtsBool(false));

  provided->AddCommandReadState(StateTable, VF_config->VF_joint_vel_limits_data.LowerLimits, "LowerJointVelLimits");
  provided->AddCommandReadState(StateTable, VF_config->VF_joint_vel_limits_data.UpperLimits, "UpperJointVelLimits");

  provided->AddCommandReadState(StateTable, VF_config->VF_absolute_joint_limits_data.LowerLimits, "LowerAbsoluteJointLimits");
  provided->AddCommandReadState(StateTable, VF_config->VF_absolute_joint_limits_data.UpperLimits, "UpperAbsoluteJointLimits");
  provided->AddCommandReadState(StateTable, this->rcm, "RCM");
  provided->AddCommandReadState(StateTable, this->rcm, "Axis"); //TODO FIX THIS 
  provided->AddCommandReadState(StateTable, VF_config->VF_follow_data.gains, "Gains");
  provided->AddCommandReadState(StateTable, this->JointVelocityCommand, "JointVelocityCommand");
  provided->AddCommandReadState(StateTable, system->system_state, "State");
  provided->AddCommandReadState(StateTable, system->system_state.CartesianPosition(), "CartesianPosition");
  provided->AddCommandReadState(StateTable, system->system_state.CartesianPositionGoal(), "CartesianPositionGoal");
  provided->AddCommandReadState(StateTable, this->next_interp_goal_vctDoubleVec, "InterpCartesianPositionGoal");
  provided->AddCommandReadState(StateTable, this->status, "Status");
  provided->AddCommandReadState(StateTable, VF_config->VF_follow_data.weights, "GetJacobianWeights");
  provided->AddCommandReadState(StateTable, system->kinematics_state.Jacobian, "GetJacobian");
  provided->AddCommandReadState(StateTable, system->system_measured_cp, "system_measured_cp");
  provided->AddCommandWrite(&mtsSARobSysTask::SetRCM, this, "SetRCM");

  // goals
  provided->AddCommandVoid(&mtsSARobSysTask::ClearGoal, this, "ClearGoal");
  provided->AddCommandVoid(&mtsSARobSysTask::PopGoal, this, "PopGoal");
  provided->AddCommandWrite(&mtsSARobSysTask::SetGoal, this, "SetGoal");
  provided->AddCommandWrite(&mtsSARobSysTask::AddGoals, this, "AddGoals");
  provided->AddCommandWrite(&mtsSARobSysTask::MoveGoal, this, "MoveGoal");
  

    // TEST
  // this->StateTable.AddData(pos_set_jp, "URInterface_move_jp");
  crtk_provided->AddCommandWrite(&mtsSARobSysTask::SetGoal, this, "/move_cp");
  // rostopic pub /crtk_interface/move_cp geometry_msgs/TraformStamped "{header: auto,  transform: {translation: {x: 1, y: 2, z: 0}, rotation: {x: 0,y: 0,z: 1, w: 0}}}"

}

void mtsSARobSysTask::SetControllerEnabled(const mtsBool &e)
{
  status = e.Data ? IDLE : DISABLED;

}

void mtsSARobSysTask::SelectConstraints(const vctIntVec &toggles)
{
  if (toggles.size() < 6)
  {
    CMN_LOG_CLASS_RUN_ERROR << "invalid attempt at selecting constraints! size is " << toggles.size() << std::endl;
    return;
  }
  CMN_LOG_CLASS_RUN_WARNING << "selecting constraints!" << std::endl;
  // VF_config->VF_rcm_active = toggles[0];
//   VF_config->VF_plane_active = toggles[1];
   VF_config->VF_absolute_joint_limits_active = toggles[2];
   VF_config->VF_joint_vel_limits_active = toggles[3];

  // VF_config->VF_sna_objective_active = toggles[4];
  // VF_config->VF_sna_constraint_active = toggles[5];
}



void mtsSARobSysTask::ClearGoal()
{
  CMN_LOG_CLASS_RUN_DEBUG << "clearing all goals!" << std::endl;
  goalpoints = queueType();
  system->StopMovement();
}

void mtsSARobSysTask::PopGoal()
{
  if (goalpoints.empty())
  {
    CMN_LOG_CLASS_RUN_ERROR << "tried to pop nonexistent goal" << std::endl;
    return;
  }
  CMN_LOG_CLASS_RUN_DEBUG << "popping goal" << std::endl;
  goalpoints.pop();
  if (goalpoints.empty())
    this->stopMovement();
}

void mtsSARobSysTask::SetGoal(const vct6 &p)
{
  this->ClearGoal();
  size_t goal_size = 6;
  vctMat pm(1, goal_size);
  for(size_t i=0; i<goal_size; i++)
    {pm(0, i) = p(i);}
  this->AddGoals(pm);
}

void mtsSARobSysTask::AddGoals(const vctMat &m)
{
  int num_goals = m.rows();
  vct6 p;
  for (int i = 0; i < num_goals; i++)
  {
    for (size_t j = 0; j < p.size(); j++) 
      { p(j) = m(i, j); }
    goalpoints.push(p);
  }
}

void mtsSARobSysTask::MoveGoal(const vct6 &p)
{
  if (goalpoints.empty())
  {
    CMN_LOG_CLASS_RUN_ERROR << "tried to move nonexistent goal in controller task" << std::endl;
    return;
  }
  goalpoints.front() = goalpoints.front() + p;
  CMN_LOG_CLASS_RUN_DEBUG << "moving current goal to " << goalpoints.front() << std::endl;
}

void mtsSARobSysTask::SetPercentFullSpeed(const mtsDouble &d)
{
  percentFullSpeed = d.Data;
}

void mtsSARobSysTask::SetPrintProblem(const mtsBool &p)
{
  CMN_LOG_CLASS_RUN_DEBUG << "setting print problem to " << p.Data << std::endl;
  printProblem = p.Data;
}

void mtsSARobSysTask::SetMarkerOffset(const vct3 &offset) // TODO: connect to VF? (or remove)
{
  CMN_LOG_CLASS_RUN_DEBUG << "setting marker offset" << std::endl;
  this->markerOffset = offset;
}

void mtsSARobSysTask::SetRCM(const vct3 &p) // TODO: connect to VF
{
  this->rcm = p;
  CMN_LOG_CLASS_RUN_DEBUG << "set rcm to " << this->rcm << std::endl;
}

 void mtsSARobSysTask::SetLowerJointVelLimits(const vctDoubleVec &lower)
{
  VF_config->VF_joint_vel_limits_data.LowerLimits = lower;
  CMN_LOG_RUN_DEBUG << "setting lower vel limits" << std::endl;
  CMN_LOG_RUN_DEBUG << VF_config->VF_joint_vel_limits_data.LowerLimits << std::endl;
}
 void mtsSARobSysTask::SetUpperJointVelLimits(const vctDoubleVec &upper)
{
  VF_config->VF_joint_vel_limits_data.UpperLimits = upper;
  CMN_LOG_RUN_DEBUG << "setting upper vel limits" << std::endl;
  CMN_LOG_RUN_DEBUG << VF_config->VF_joint_vel_limits_data.UpperLimits << std::endl;
}

void mtsSARobSysTask::SetLowerAbsoluteJointLimits(const vctDoubleVec &lower) //TODO:
{
  VF_config->VF_absolute_joint_limits_data.LowerLimits = lower;
  CMN_LOG_RUN_DEBUG << "setting lower abs limits" << std::endl;
  CMN_LOG_RUN_DEBUG << VF_config->VF_absolute_joint_limits_data.LowerLimits << std::endl;
}

void mtsSARobSysTask::SetUpperAbsoluteJointLimits(const vctDoubleVec &upper)
{
  VF_config->VF_absolute_joint_limits_data.UpperLimits = upper;
  CMN_LOG_RUN_DEBUG << "setting upper abs limits" << std::endl;
  CMN_LOG_RUN_DEBUG << VF_config->VF_absolute_joint_limits_data.UpperLimits << std::endl;
}

void mtsSARobSysTask::SetAxis(const vct3 &a) //TODO: Re-implement
{
  std::cout << "Axis setting is currently not implemented" << std::endl;
  // Sensors[AXIS_SENSOR]->Values = vctVec(a);
  // CMN_LOG_RUN_DEBUG << "set normal to" <<std::endl;
  // CMN_LOG_RUN_DEBUG << Sensors[AXIS_SENSOR]->Values << std::endl;
}

void mtsSARobSysTask::SetGains(const vct3 &g)
{
  VF_config->VF_follow_data.gains = g;
  CMN_LOG_RUN_DEBUG << "set gains to" <<std::endl;
  CMN_LOG_RUN_DEBUG << VF_config->VF_follow_data.gains << std::endl;
}

void mtsSARobSysTask::SetJacobianWeights(const vctMat &w)
{
  VF_config->VF_follow_data.weights = w;
  CMN_LOG_RUN_DEBUG << "set weights to" << std::endl;
  CMN_LOG_RUN_DEBUG << VF_config->VF_follow_data.weights << std::endl;
}

void mtsSARobSysTask::SetGamma(const mtsDouble &g) //TODO:reimplement or remove
{}
