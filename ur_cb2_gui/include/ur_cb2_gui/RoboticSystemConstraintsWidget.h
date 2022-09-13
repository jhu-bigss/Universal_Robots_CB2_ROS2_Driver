#ifndef _RoboticSystemConstraintsWidget_h_
#define _RoboticSystemConstraintsWidget_h_

#include <QWidget>
#include <QSlider>
#include <cisstMultiTask/mtsComponent.h>

//class ctkPathLineEdit;
class QDoubleSpinBox;
class QCheckBox;
class QPushButton;
//class ctkSliderWidget;

namespace Ui
{
  class demoConstraintsWidget;
}

class RoboticSystemConstraintsWidget
    : public QWidget, public mtsComponent
{

  Q_OBJECT

  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

  RoboticSystemConstraintsWidget(int num_joints,
    const std::string &name="RoboticSystemConstraintsWidget",
    QWidget *parent=0
  );

  virtual ~RoboticSystemConstraintsWidget() {};

  // this should be called after the controller has been started, so that it is
  // availabled via multitask
  void updateWidgetValues();
  void syncConstraints();

protected:

  void setupMultiTask();

protected slots:

  void On_velocityBoxes_Changed();
  void On_limitBoxes_Changed();
  void On_activeBoxes_Toggled();
  void On_rcmBoxes_Changed();
  void On_axisBoxes_Changed();
  void On_gainsBoxes_Changed();
  void On_filterBoxes_Changed();
  void On_jacobianWeightsBoxes_Changed();

protected:

  size_t numJoints;
  size_t numConstraints;
  size_t numJointGroups;

  Ui::demoConstraintsWidget *ui;

  std::vector<QDoubleSpinBox*> lowerVelocity;
  std::vector<QDoubleSpinBox*> upperVelocity;

  std::vector<QDoubleSpinBox*> lowerLimit;
  std::vector<QDoubleSpinBox*> upperLimit;

  std::vector<QDoubleSpinBox*> jacobianWeights;

  std::vector<QSlider*> weights;

  std::vector<QCheckBox*> active;

  std::vector<int> jointGroups;

  mtsFunctionRead GetLowerJointVelLimits;
  mtsFunctionRead GetUpperJointVelLimits;
  mtsFunctionRead GetLowerAbsoluteJointLimits;
  mtsFunctionRead GetUpperAbsoluteJointLimits;
  mtsFunctionRead GetRCM;
  mtsFunctionRead GetFilterParams;
  mtsFunctionRead GetAxis;
  mtsFunctionRead GetGains;
  mtsFunctionRead GetJacobianWeights;

  mtsFunctionWrite SetLowerJointVelLimits;
  mtsFunctionWrite SetUpperJointVelLimits;
  mtsFunctionWrite SetLowerAbsoluteJointLimits;
  mtsFunctionWrite SetUpperAbsoluteJointLimits;
  mtsFunctionWrite SetRCM;
  mtsFunctionWrite SetAxis;
  mtsFunctionWrite SetGains;
  mtsFunctionWrite SetFilterParams;
  mtsFunctionWrite SetJacobianWeights;
 
  mtsFunctionWrite SelectConstraints;

};

CMN_DECLARE_SERVICES_INSTANTIATION(RoboticSystemConstraintsWidget);

#endif

