
#include <bigssRoboticSystem/RoboticSystemStatusWidget.h>

#include <bigssRoboticSystem/ui_RoboticSystemStatusWidget.h>

#include <bigssRoboticSystem/mtsSARobSysTask.h>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassServices.h>
#include <cisstCommon/cmnClassRegisterMacros.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstMultiTask/mtsMatrix.h>

CMN_IMPLEMENT_SERVICES_DERIVED(RoboticSystemStatusWidget, mtsComponent);


RoboticSystemStatusWidget::RoboticSystemStatusWidget(
  const std::string &name, QWidget *parent, int num_joints)
  : QWidget(parent), mtsComponent(name), ui(new Ui::demoStatusWidget()), numJoints(num_joints)
{
  this->setupMultiTask();
  ui->setupUi(this);

  velocityLabels.push_back(ui->commandedVelocity0Label);
  velocityLabels.push_back(ui->commandedVelocity1Label);
  velocityLabels.push_back(ui->commandedVelocity2Label);
  velocityLabels.push_back(ui->commandedVelocity3Label);
  velocityLabels.push_back(ui->commandedVelocity4Label);
  velocityLabels.push_back(ui->commandedVelocity5Label);

  stateLabels.push_back(ui->state0Label);
  stateLabels.push_back(ui->state1Label);
  stateLabels.push_back(ui->state2Label);
  stateLabels.push_back(ui->state3Label);
  stateLabels.push_back(ui->state4Label);
  stateLabels.push_back(ui->state5Label);

  //TODO: this is a temp fix for having variable number of joints not break UI, but needs to be better implemented
  int extra_joints = numJoints-velocityLabels.size();
  if(extra_joints){
    for(size_t i=0; i<extra_joints; i++){
      QLabel* label = new QLabel();
      label->setText("0.0000");
      ui->commandedVelocitiesLayout->addWidget(label);
      velocityLabels.push_back(label);
    }
    for(size_t i=0; i<extra_joints; i++){
      QLabel* label = new QLabel();
      label->setText("0.0000");
      ui->stateLayout->addWidget(label);
      stateLabels.push_back(label);
    }
  }

}

void RoboticSystemStatusWidget::setupMultiTask()
{
  mtsInterfaceRequired *required = this->AddInterfaceRequired("userInterface");
  if (required)
  {
    required->AddFunction("State", this->GetState);
    required->AddFunction("Status", this->GetStatus);
    required->AddFunction("SetPrintProblem", this->SetPrintProblem);
    // required->AddFunction("GetPolarisBase", this->GetPolarisBase);
    // required->AddFunction("GetFKTooltip", this->GetFKTooltip);
  }
/*
  required = this->AddInterfaceRequired("medtronicInterface");
  if (required)
  {
    required->AddFunction("GetPositionCartesian", this->GetMedtronicPose);
  }
*/
}

void RoboticSystemStatusWidget::update()
{
  CMN_LOG_CLASS_RUN_DEBUG << "updating status widget" << std::endl;
  prmRobotState state;
  this->GetState(state);

  int status;
  this->GetStatus(status);

  vct6 currCartPos = state.CartesianPosition();

  vct6 currCartGoal = state.CartesianPositionGoal();
  ui->currentMeasuredTip0->setText(this->doubleToQString(currCartPos(0)));
  ui->currentMeasuredTip1->setText(this->doubleToQString(currCartPos(1)));
  ui->currentMeasuredTip2->setText(this->doubleToQString(currCartPos(2)));
  ui->currentMeasuredTip3->setText(this->doubleToQString(currCartPos(3)));
  ui->currentMeasuredTip4->setText(this->doubleToQString(currCartPos(4)));
  ui->currentMeasuredTip5->setText(this->doubleToQString(currCartPos(5)));

  ui->currentGoal0->setText(this->doubleToQString(currCartGoal(0)));
  ui->currentGoal1->setText(this->doubleToQString(currCartGoal(1)));
  ui->currentGoal2->setText(this->doubleToQString(currCartGoal(2)));
  ui->currentGoal3->setText(this->doubleToQString(currCartGoal(3)));
  ui->currentGoal4->setText(this->doubleToQString(currCartGoal(4)));
  ui->currentGoal5->setText(this->doubleToQString(currCartGoal(5)));

  ui->distanceLabel->setText(QString::fromStdString(
    std::to_string(state.CartesianPositionError().Norm())));

  // controller status
  switch (status)
  {
    case 0: // todo: wrap the status enum with mtsGenericObjectProxy
      ui->statusLabel->setText(QString("DISABLED")); break;
    case 1:
      ui->statusLabel->setText(QString("IDLE")); break;
    case 2:
      ui->statusLabel->setText(QString("RUNNING")); break;
    case 3:
      ui->statusLabel->setText(QString("INFEASIBLE")); break;
  }

  for (int i = 0; i < numJoints; i++)
  {
    velocityLabels[i]->setText(this->doubleToQString(state.JointVelocityGoal()[i]));
  }

  for (int i = 0; i < numJoints; i++)
  {
    if(state.JointPosition().size()!=numJoints){
      std::cout << "Status widget waiting for joint states" << std::endl;
      break;
    }
    stateLabels[i]->setText(this->doubleToQString(state.JointPosition()[i]));
  }

  // update the polaris position
  // vctFrm3 FKTooltip;
  // this->GetFKTooltip(FKTooltip);

  // ui->fkXLabel->setText(this->doubleToQString(FKTooltip.Translation().X()));
  // ui->fkYLabel->setText(this->doubleToQString(FKTooltip.Translation().Y()));
  // ui->fkZLabel->setText(this->doubleToQString(FKTooltip.Translation().Z()));

  // prmPositionCartesianGet PolarisBase;
  // this->GetPolarisBase(PolarisBase);
  // //std::cout << PolarisBase << std::endl;
  // ui->polarisBaseXLabel->setText(this->doubleToQString(PolarisBase.Position().Translation().X()));
  // ui->polarisBaseYLabel->setText(this->doubleToQString(PolarisBase.Position().Translation().Y()));
  // ui->polarisBaseZLabel->setText(this->doubleToQString(PolarisBase.Position().Translation().Z()));
}

void RoboticSystemStatusWidget::on_printProblemBox_stateChanged(int state)
{
  bool s = false;
  if (state > 0) s = true;
  this->SetPrintProblem(s);
}

QString RoboticSystemStatusWidget::doubleToQString(const double &d)
{
  std::string str = std::to_string(d);
  return QString::fromStdString(str);
}

