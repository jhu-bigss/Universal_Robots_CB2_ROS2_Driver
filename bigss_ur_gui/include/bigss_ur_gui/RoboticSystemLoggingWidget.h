#ifndef _RoboticSystemLoggingWidget_h_
#define _RoboticSystemLoggingWidget_h_

#include <QWidget>
#include <cisstMultiTask/mtsComponent.h>

class qMRMLNodeComboBox;
class QPushButton;

class vtkMRMLScene;
class vtkMRMLNode;

namespace Ui
{
  class demoLoggingWidget;
}

class RoboticSystemLoggingWidget
    : public QWidget, public mtsComponent
{
  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

  RoboticSystemLoggingWidget(
    const std::string &name="RoboticSystemLoggingWidget",
    QWidget *parent=0
  );
  virtual ~RoboticSystemLoggingWidget() {};

public slots:

  void updateSelections(vtkMRMLScene *scene);
  void on_loggingFiducialsSelectionBox_currentNodeChanged(vtkMRMLNode *node);
  void on_loggingRecordButton_clicked();
  void on_loggingRecordIntervalBox_valueChanged(double d);
  void on_loggingAutoRecordButton_toggled(bool checked);
  void on_loggingClearPointsButton_clicked();
  void on_recordTimer_timeout();

private:

  double recordInterval;
  QTimer *recordTimer;

  Ui::demoLoggingWidget *ui;

};

CMN_DECLARE_SERVICES_INSTANTIATION(RoboticSystemLoggingWidget);

#endif

