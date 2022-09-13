#ifndef _RoboticSystemControlWidget_h_
#define _RoboticSystemControlWidget_h_

#include <QWidget>
#include <cisstMultiTask/mtsComponent.h>
#include <vector>

class QPushButton;
class QDoubleSpinBox;
class QCheckBox;
class QListWidget;
class QListWidgetItem;
//class ctkSliderWidget;
class QSlider; 

class mtsCollectorState;

namespace Ui
{
  class demoControlWidget;
}

class RoboticSystemControlWidget
    : public QWidget, public mtsComponent
{
  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)

public:
  RoboticSystemControlWidget(
    const std::string &pts_filename="points.txt",
    const std::string &name="RoboticSystemControlWidget",
    QWidget *parent=0
  );
  virtual ~RoboticSystemControlWidget() {};

  mtsFunctionVoid PopGoal;
  mtsFunctionVoid ClearGoal;
  mtsFunctionWrite AddGoals;
  mtsFunctionWrite SetGoal;
  mtsFunctionWrite MoveGoal;
  mtsFunctionWrite SetPercentFullSpeed;
  mtsFunctionWrite setDigitalOutput;
  mtsFunctionRead GetCartesianPosition;
  mtsFunctionWrite SetControllerEnabled;

  void addCollector(mtsManagerLocal *lm, const std::string &taskName, const std::string &stateTableName, const std::vector<std::string> &s); 
  void startCollectors();
  void stopCollectors();

private slots:

  void on_demoCuttingButton_clicked();

  void on_trajectoryMoveNextButton_clicked();
  void on_trajectoryMoveAutoButton_clicked();

  void on_goalPointNegativeXButton_clicked();
  void on_goalPointNegativeYButton_clicked();
  void on_goalPointNegativeZButton_clicked();
  void on_goalPointPositiveXButton_clicked();
  void on_goalPointPositiveYButton_clicked();
  void on_goalPointPositiveZButton_clicked();

  void on_demoSpeedSlider_valueChanged(int d);
  void on_loadPointsButton_clicked();
  void on_savePointsButton_clicked();  
  void on_addCurrentToGoalListButton_clicked();
  void on_enable_button_clicked();
  void on_disable_button_clicked();


  void on_goalPointGoButton_clicked();
  void on_goalPointStopButton_clicked();
  void on_resetGoalPointButton_clicked();

  void on_loggingBox_stateChanged();

  void on_clearPointsButton_clicked();
  void on_resetToInitPosButton_clicked();

private:

  Ui::demoControlWidget *ui;

  std::vector<mtsCollectorState*> collectors;

  void setupMultiTask();

  void setCurrentListItemToPoint(const vct6 &v);
  bool listItemToPoint(QListWidgetItem *item, vct6 &v);
  void pointToListItem(const vct6 &p, QListWidgetItem *item);
  std::string pointToStdString(const vct6 &p);
  vct6 stringToPoint(const std::string &s);
  std::string points_filename;
  vctDoubleVec cartesian_position;

};

CMN_DECLARE_SERVICES_INSTANTIATION(RoboticSystemControlWidget)

#endif

