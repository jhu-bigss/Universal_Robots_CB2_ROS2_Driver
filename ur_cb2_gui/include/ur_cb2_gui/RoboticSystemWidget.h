#ifndef _UR5CONCURRENTCONTROLPANEL_H
#define _UR5CONCURRENTCONTROLPANEL_H

#include <QWidget>
#include <QTabWidget>

class RoboticSystemInputsWidget;
class RoboticSystemControlWidget;
class RoboticSystemConstraintsWidget;
// class RoboticSystemLoggingWidget;
class RoboticSystemStatusWidget;

class vtkMRMLScene;
//class ctkCollapsibleButton;

class RoboticSystemWidget : public QWidget
{
Q_OBJECT

public:

  RoboticSystemWidget(
    std::string points_filename="points.txt",
    int num_joints=0,
    QWidget *parent=0
  );
  virtual ~RoboticSystemWidget();

  RoboticSystemInputsWidget *inputs_;
  RoboticSystemControlWidget *control_;
  RoboticSystemConstraintsWidget *constraints_;
  // RoboticSystemLoggingWidget *logging_;
  RoboticSystemStatusWidget *status_;

private slots:

  void start_controller();
  void stop_controller();
  void on_timer_timeout();

protected:

  QTimer *timer_;
  QTabWidget  *tabs_;
  //QTabWidget *inputs_button_;
  // ctkCollapsibleButton *logging_button_;
  //QTabWidget *control_button_;
  //QTabWidget *constraints_button_;
  //QTabWidget *status_button_;

  //QTabWidget* wrapInDropdownButton(const std::string &title, QWidget *widget);

  void enable_controller_buttons();
  void disable_controller_buttons();

};

#endif

