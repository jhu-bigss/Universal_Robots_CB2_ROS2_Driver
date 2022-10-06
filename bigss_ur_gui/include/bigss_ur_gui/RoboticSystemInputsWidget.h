#ifndef _RoboticSystemInputsWidget_h_
#define _RoboticSystemInputsWidget_h_

#include <QWidget>
#include <cisstMultiTask/mtsComponent.h>

class QPushButton;
class QLineEdit;

class RoboticSystemInputsWidget : public QWidget, public mtsComponent
{

  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
  RoboticSystemInputsWidget(
      const std::string &name="RoboticSystemInputsWidget", QWidget *parent=0);
  virtual ~RoboticSystemInputsWidget() {};

  void enableController();
  void disableController();

  void perform_initialization(const std::string &tool_file, const std::string &marker_off_file);

protected:
  void setupMultiTask();

  void setupUi();

  bool setTransforms();
  bool loadTransformFromSelectionBox(QLineEdit *sel, vctFrm3 &frame);

  void enable_inputs();
  void disable_inputs();

signals:
  void controller_enabled();
  void controller_disabled();

private slots:
  void setup_button_clicked();

protected:

  /// Inputs
  QLineEdit* tool_off_xform_sel_;
  QLineEdit* marker_offset_sel_;

  // setup button
  QPushButton *setup_button_;

  mtsFunctionWrite SetToolOffset;
  mtsFunctionWrite SetMarkerOffset;
  mtsFunctionWrite SetControllerEnabled;

};

CMN_DECLARE_SERVICES_INSTANTIATION(RoboticSystemInputsWidget);

#endif

