
#ifndef _RoboticSystemStatusWidget_h_
#define _RoboticSystemStatusWidget_h_

#include <QWidget>
#include <QFormLayout>
#include <QPushButton>
#include <QLabel>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstParameterTypes/prmRobotState.h>

class QDoubleSpinBox;
class QPushButton;

namespace Ui
{
  class demoStatusWidget;
}

class RoboticSystemStatusWidget
    : public QWidget, public mtsComponent
{
  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
  RoboticSystemStatusWidget(
    const std::string &name="RoboticSystemStatusWidget",
    QWidget *parent=0,
    int num_joints = 0
  );
  virtual ~RoboticSystemStatusWidget() {};

  QString doubleToQString(const double &d);

  void update();

private:

  Ui::demoStatusWidget *ui;

  int numJoints;
  std::vector<QLabel*> velocityLabels;
  std::vector<QLabel*> velocityFilteredLabels;
  std::vector<QLabel*> stateLabels;

  mtsFunctionRead GetJointVelocityGoalFiltered;
  mtsFunctionRead GetState;
  mtsFunctionRead GetStatus;
  mtsFunctionWrite SetPrintProblem;

  mtsFunctionRead GetPolarisTip;
  mtsFunctionRead GetFKTooltip;
  mtsFunctionRead GetPolarisBase;

  mtsFunctionRead GetMedtronicPose;

protected:

  void setupMultiTask();
  void disable_inputs();
  void enable_inputs();

protected slots:

  void on_printProblemBox_stateChanged(int state);

};

CMN_DECLARE_SERVICES_INSTANTIATION(RoboticSystemStatusWidget);

#endif

