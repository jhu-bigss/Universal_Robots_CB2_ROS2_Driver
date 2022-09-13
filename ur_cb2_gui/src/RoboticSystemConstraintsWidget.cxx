#include <ur_cb2_gui/RoboticSystemConstraintsWidget.h>

#include <QLabel>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QGridLayout>

//#include <ctkPathLineEdit.h>

#include <ur_cb2_gui/ui_RoboticSystemConstraintsWidget.h>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassServices.h>
#include <cisstCommon/cmnClassRegisterMacros.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED(RoboticSystemConstraintsWidget, mtsComponent);

RoboticSystemConstraintsWidget::RoboticSystemConstraintsWidget(int num_joints,
  const std::string &name,
  QWidget *parent
  ) :
  QWidget(parent),
  mtsComponent(name),
  numJoints(num_joints),
  numConstraints(6),
  numJointGroups(4),
  ui(new Ui::demoConstraintsWidget())
{
  this->setupMultiTask();
  ui->setupUi(this);

  // get pointers for ease of access
  lowerVelocity.push_back(ui->lowerVelocityJoint0Box);
  lowerVelocity.push_back(ui->lowerVelocityJoint1Box);
  lowerVelocity.push_back(ui->lowerVelocityJoint2Box);
  lowerVelocity.push_back(ui->lowerVelocityJoint3Box);
  lowerVelocity.push_back(ui->lowerVelocityJoint4Box);
  lowerVelocity.push_back(ui->lowerVelocityJoint5Box);
  // lowerVelocity.push_back(ui->lowerVelocityJoint6Box);
  // lowerVelocity.push_back(ui->lowerVelocityJoint7Box);

  // get pointers for ease of access
  upperVelocity.push_back(ui->upperVelocityJoint0Box);
  upperVelocity.push_back(ui->upperVelocityJoint1Box);
  upperVelocity.push_back(ui->upperVelocityJoint2Box);
  upperVelocity.push_back(ui->upperVelocityJoint3Box);
  upperVelocity.push_back(ui->upperVelocityJoint4Box);
  upperVelocity.push_back(ui->upperVelocityJoint5Box);
  // upperVelocity.push_back(ui->upperVelocityJoint6Box);
  // upperVelocity.push_back(ui->upperVelocityJoint7Box);

  // get pointers for ease of access
  lowerLimit.push_back(ui->lowerLimitsJoint0Box);
  lowerLimit.push_back(ui->lowerLimitsJoint1Box);
  lowerLimit.push_back(ui->lowerLimitsJoint2Box);
  lowerLimit.push_back(ui->lowerLimitsJoint3Box);
  lowerLimit.push_back(ui->lowerLimitsJoint4Box);
  lowerLimit.push_back(ui->lowerLimitsJoint5Box);
  // lowerLimit.push_back(ui->lowerLimitsJoint6Box);
  // lowerLimit.push_back(ui->lowerLimitsJoint7Box);

  // get pointers for ease of access
  upperLimit.push_back(ui->upperLimitsJoint0Box);
  upperLimit.push_back(ui->upperLimitsJoint1Box);
  upperLimit.push_back(ui->upperLimitsJoint2Box);
  upperLimit.push_back(ui->upperLimitsJoint3Box);
  upperLimit.push_back(ui->upperLimitsJoint4Box);
  upperLimit.push_back(ui->upperLimitsJoint5Box);
  // upperLimit.push_back(ui->upperLimitsJoint6Box);
  // upperLimit.push_back(ui->upperLimitsJoint7Box);

  // get pointers for ease of access
  jacobianWeights.push_back(ui->jacobianWeightsJoint0Box);
  jacobianWeights.push_back(ui->jacobianWeightsJoint1Box);
  jacobianWeights.push_back(ui->jacobianWeightsJoint2Box);
  jacobianWeights.push_back(ui->jacobianWeightsJoint3Box);
  jacobianWeights.push_back(ui->jacobianWeightsJoint4Box);
  jacobianWeights.push_back(ui->jacobianWeightsJoint5Box);


//TODO: this is a temp fix for 8 joints not break UI, but needs to be better implemented
  if(numJoints > 6){
    lowerVelocity.push_back(ui->lowerVelocityJoint6Box);
    upperVelocity.push_back(ui->upperVelocityJoint6Box);
    lowerLimit.push_back(ui->lowerLimitsJoint6Box);
    upperLimit.push_back(ui->upperLimitsJoint6Box);
    jacobianWeights.push_back(ui->jacobianWeightsJoint6Box);
  }
  if(numJoints > 7){
    lowerVelocity.push_back(ui->lowerVelocityJoint7Box);
    upperVelocity.push_back(ui->upperVelocityJoint7Box);
    lowerLimit.push_back(ui->lowerLimitsJoint7Box);
    upperLimit.push_back(ui->upperLimitsJoint7Box);
    jacobianWeights.push_back(ui->jacobianWeightsJoint7Box);
  }

  // jacobianWeights.push_back(ui->jacobianWeightsJoint6Box);
  // jacobianWeights.push_back(ui->jacobianWeightsJoint7Box);

  // jointGroups.resize(numJoints); //TODO:Fix joint groups
  // jointGroups[0] = 0;
  // jointGroups[1] = 0;
  // jointGroups[2] = 0;
  // jointGroups[3] = 1;
  // jointGroups[4] = 1;
  // jointGroups[5] = 1;
  // jointGroups[6] = 2;
  // jointGroups[7] = 3;

  // set weights on degrees of freedom
  weights.push_back(ui->jointGroup1Slider);
  weights.push_back(ui->jointGroup2Slider);
  weights.push_back(ui->rollMotorSlider);
  weights.push_back(ui->snakeMotorSlider);

  // list of active boxes for selecting constraints
  active.push_back(ui->rcmActiveBox);
  active.push_back(ui->planeActiveBox);
  active.push_back(ui->limitsActiveBox);
  active.push_back(ui->velocityActiveBox);
  active.push_back(ui->snaActiveBox);
  active.push_back(ui->snaConstraintActiveBox);

  // connect signals
  for (size_t i = 0; i < numJoints; i++)
  {
    connect(lowerVelocity[i], SIGNAL(valueChanged(double)), this, SLOT(On_velocityBoxes_Changed()));
    connect(upperVelocity[i], SIGNAL(valueChanged(double)), this, SLOT(On_velocityBoxes_Changed()));
    connect(lowerLimit[i], SIGNAL(valueChanged(double)), this, SLOT(On_limitBoxes_Changed()));
    connect(upperLimit[i], SIGNAL(valueChanged(double)), this, SLOT(On_limitBoxes_Changed()));
    connect(jacobianWeights[i], SIGNAL(valueChanged(double)), this, SLOT(On_jacobianWeightsBoxes_Changed()));
  }

  for (size_t i = 0; i < numJointGroups; i++) // TODO: Fix #s
    connect(weights[i], SIGNAL(valueChanged(int)), this, SLOT(On_velocityBoxes_Changed()));

  for (size_t i = 0; i < numConstraints; i++)
    connect(active[i], SIGNAL(stateChanged(int)), this, SLOT(On_activeBoxes_Toggled()));

  connect(ui->rcmXBox, SIGNAL(valueChanged(double)), this, SLOT(On_rcmBoxes_Changed()));
  connect(ui->rcmYBox, SIGNAL(valueChanged(double)), this, SLOT(On_rcmBoxes_Changed()));
  connect(ui->rcmZBox, SIGNAL(valueChanged(double)), this, SLOT(On_rcmBoxes_Changed()));

  connect(ui->axisXBox, SIGNAL(valueChanged(double)), this, SLOT(On_axisBoxes_Changed()));
  connect(ui->axisYBox, SIGNAL(valueChanged(double)), this, SLOT(On_axisBoxes_Changed()));
  connect(ui->axisZBox, SIGNAL(valueChanged(double)), this, SLOT(On_axisBoxes_Changed()));

  connect(ui->proportionalGainBox, SIGNAL(valueChanged(double)), this, SLOT(On_gainsBoxes_Changed()));
  connect(ui->derivativeGainBox, SIGNAL(valueChanged(double)), this, SLOT(On_gainsBoxes_Changed()));
}

void RoboticSystemConstraintsWidget::updateWidgetValues()
{
  vctDoubleVec lowerVelInit(numJoints, 0.0);
  this->GetLowerJointVelLimits(lowerVelInit);

  vctDoubleVec upperVelInit(numJoints, 0.0);
  this->GetUpperJointVelLimits(upperVelInit);

  vctDoubleVec lowerAbsInit(numJoints, 0.0); 
  this->GetLowerAbsoluteJointLimits(lowerAbsInit);

  vctDoubleVec upperAbsInit(numJoints, 0.0);
  this->GetUpperAbsoluteJointLimits(upperAbsInit);

  // vctMat weights;
  // this->GetJacobianWeights(weights);

  for (size_t i = 0; i < numJoints; i++)
  {

    lowerVelocity[i]->blockSignals(true);
    lowerVelocity[i]->setValue(lowerVelInit.Element(i));
    lowerVelocity[i]->blockSignals(false);

    upperVelocity[i]->blockSignals(true);
    upperVelocity[i]->setValue(upperVelInit.Element(i));
    upperVelocity[i]->blockSignals(false);

    lowerLimit[i]->blockSignals(true);
    lowerLimit[i]->setValue(lowerAbsInit.Element(i));
    lowerLimit[i]->blockSignals(false);

    upperLimit[i]->blockSignals(true);
    upperLimit[i]->setValue(upperAbsInit.Element(i));
    upperLimit[i]->blockSignals(false);

    // jacobianWeights[i]->blockSignals(true);
    // jacobianWeights[i]->setValue(weights(i, i));
    // jacobianWeights[i]->blockSignals(false);
  }

  // set rcm location
  vct3 rcm;
  this->GetRCM(rcm);

  // std::cout << "current rcm is " << std::endl << rcm << std::endl;
  // std::cout << "initial widget values are " << ui->rcmXBox->value() << ", "
  //   << ui->rcmYBox->value() << ", " << ui->rcmZBox->value() << std::endl;

  ui->rcmXBox->blockSignals(true);
  ui->rcmXBox->setValue(rcm.X());
  ui->rcmXBox->blockSignals(false);

  ui->rcmYBox->blockSignals(true);
  ui->rcmYBox->setValue(rcm.Y());
  ui->rcmYBox->blockSignals(false);

  ui->rcmZBox->blockSignals(true);
  ui->rcmZBox->setValue(rcm.Z());
  ui->rcmZBox->blockSignals(false);

  // plane orientation
  vctVec planeNormal;

  //TODO: REMOVE HARDCODE
  planeNormal.SetSize(3);

  this->GetAxis(planeNormal);
  // std::cout << "planeNormal: " << planeNormal << std::endl;
  ui->axisXBox->blockSignals(true);
  ui->axisXBox->setValue(planeNormal[0]);
  ui->axisXBox->blockSignals(false);

  ui->axisYBox->blockSignals(true);
  ui->axisYBox->setValue(planeNormal[1]);
  ui->axisYBox->blockSignals(false);

  ui->axisZBox->blockSignals(true);
  ui->axisZBox->setValue(planeNormal[2]);
  ui->axisZBox->blockSignals(false);

  vct3 gains;
  this->GetGains(gains);

  ui->proportionalGainBox->blockSignals(true);
  ui->proportionalGainBox->setValue(gains.X());
  ui->proportionalGainBox->blockSignals(false);

  ui->derivativeGainBox->blockSignals(true);
  ui->derivativeGainBox->setValue(gains.Y());
  ui->derivativeGainBox->blockSignals(false);

  // this one behaves a bit differently: it just sets the initial values
  // how the widget has it. So, the initial values of the constraint
  // parameters are determined by how they are set in mtsURPolarisCLVelOptController,
  // while the initial active booleans are set by the initial widget
  // settings.
  this->syncConstraints();
}

void RoboticSystemConstraintsWidget::setupMultiTask()
{
  mtsInterfaceRequired *required = this->AddInterfaceRequired("userInterface");
  if (!required)
  {
    CMN_LOG_CLASS_RUN_ERROR << "Unabled to connect constraints widget to snake task!" << std::endl;
  }
  required->AddFunction("LowerJointVelLimits", this->GetLowerJointVelLimits);
  required->AddFunction("UpperJointVelLimits", this->GetUpperJointVelLimits);
  required->AddFunction("LowerAbsoluteJointLimits", this->GetLowerAbsoluteJointLimits);
  required->AddFunction("UpperAbsoluteJointLimits", this->GetUpperAbsoluteJointLimits);
  required->AddFunction("RCM", this->GetRCM);
  required->AddFunction("Axis", this->GetAxis);
  required->AddFunction("Gains", this->GetGains);

  required->AddFunction("SetLowerJointVelLimits", this->SetLowerJointVelLimits);
  required->AddFunction("SetUpperJointVelLimits", this->SetUpperJointVelLimits);
  required->AddFunction("SetLowerAbsoluteJointLimits", this->SetLowerAbsoluteJointLimits);
  required->AddFunction("SetUpperAbsoluteJointLimits", this->SetUpperAbsoluteJointLimits);
  required->AddFunction("SetRCM", this->SetRCM);
  required->AddFunction("SetAxis", this->SetAxis);
  required->AddFunction("SetGains", this->SetGains);

  required->AddFunction("SelectConstraints", this->SelectConstraints);
  required->AddFunction("SetJacobianWeights", this->SetJacobianWeights);
  required->AddFunction("GetJacobianWeights", this->GetJacobianWeights);
}

void RoboticSystemConstraintsWidget::On_velocityBoxes_Changed()
{
  size_t num_boxes = lowerVelocity.size();
  vctVec lower(num_boxes), upper(num_boxes);
  std::vector<bool> jointsActive;

  for (size_t i = 0; i < num_boxes; i++)
  {
    double scaleFactor = weights[jointGroups[i]]->value()/100.0;
    lower[i] = lowerVelocity[i]->value()*scaleFactor;
    upper[i] = upperVelocity[i]->value()*scaleFactor;
    jointsActive.push_back(true);
  }

  this->SetLowerJointVelLimits(lower);
  this->SetUpperJointVelLimits(upper);
}

void RoboticSystemConstraintsWidget::On_limitBoxes_Changed()
{

  size_t num_boxes = lowerLimit.size();
  vctVec lower(num_boxes), upper(num_boxes);
  std::vector<bool> jointsActive(num_boxes);

  for (size_t i = 0; i < num_boxes; i++)
  {
    lower[i] = lowerLimit[i]->value();
    upper[i] = upperLimit[i]->value();
    jointsActive.push_back(true);
  }

  this->SetLowerAbsoluteJointLimits(lower);
  this->SetUpperAbsoluteJointLimits(upper);
}

void RoboticSystemConstraintsWidget::On_activeBoxes_Toggled()
{
  this->syncConstraints();
}

void RoboticSystemConstraintsWidget::syncConstraints()
{
  size_t num_toggles = active.size();
  vctIntVec toggles(num_toggles, 0);

  CMN_LOG_CLASS_RUN_DEBUG << "selecting constraints" << std::endl;

  for (size_t i = 0; i < num_toggles; i++)
  {
    toggles[i] = active[i]->isChecked()? 1 : 0;
    CMN_LOG_CLASS_RUN_DEBUG << toggles[i] << std::endl;
  }

  this->SelectConstraints(toggles);
}

void RoboticSystemConstraintsWidget::On_filterBoxes_Changed()
{}

void RoboticSystemConstraintsWidget::On_rcmBoxes_Changed()
{
  vct3 rcm;
  rcm.X() = ui->rcmXBox->value();
  rcm.Y() = ui->rcmYBox->value();
  rcm.Z() = ui->rcmZBox->value();
  this->SetRCM(rcm);
}

void RoboticSystemConstraintsWidget::On_axisBoxes_Changed()
{
  vctVec axis(3, 0.0);
  axis[0] = ui->axisXBox->value();
  axis[1] = ui->axisYBox->value();
  axis[2] = ui->axisZBox->value();
  this->SetAxis(axis);
}

void RoboticSystemConstraintsWidget::On_gainsBoxes_Changed()
{
  vct3 gains;
  gains.X() = ui->proportionalGainBox->value();
  gains.Y() = ui->derivativeGainBox->value();
  this->SetGains(gains);
}

void RoboticSystemConstraintsWidget::On_jacobianWeightsBoxes_Changed()
{
  vctMat weights = vctMat::Eye(jacobianWeights.size());
  for (int i = 0; i < jacobianWeights.size(); i++)
  {
    weights(i, i) = jacobianWeights[i]->value();
  }

  this->SetJacobianWeights(weights);

  std::cout << "jacobian weights changed" << std::endl;
  std::cout << weights<< std::endl;
}

