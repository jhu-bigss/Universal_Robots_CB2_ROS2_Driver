#include <ur_cb2_gui/RoboticSystemInputsWidget.h>

#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
#include <QtGui>
#else 
#include <QtWidgets>
#endif

#include <QString>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassServices.h>
#include <cisstCommon/cmnClassRegisterMacros.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED(RoboticSystemInputsWidget, mtsComponent);

RoboticSystemInputsWidget::RoboticSystemInputsWidget(
    const std::string &name,
    QWidget *p)
  : QWidget(), mtsComponent(name)
{
  this->setupMultiTask();
  setupUi();
}

void RoboticSystemInputsWidget::setupMultiTask()
{
  // setup interface to the controller
  mtsInterfaceRequired *required = this->AddInterfaceRequired("userInterface");
  if (required)
  {
    // required->AddFunction("SetToolOffset", this->SetToolOffset);
    // required->AddFunction("SetMarkerOffset", this->SetMarkerOffset);
    required->AddFunction("SetControllerEnabled", this->SetControllerEnabled);
  }
}

void RoboticSystemInputsWidget::setupUi()
{
  QFormLayout* layout = new QFormLayout;
  setLayout(layout);

  tool_off_xform_sel_ = new QLineEdit;
  layout->addRow(QString::fromStdString("Tool Offset:"), tool_off_xform_sel_);

  marker_offset_sel_ = new QLineEdit;
  layout->addRow(QString::fromStdString("Marker Offset:"), marker_offset_sel_);

  setup_button_ = new QPushButton("Set Inputs");
  connect(setup_button_, SIGNAL(released()), this,
      SLOT(setup_button_clicked()));

  layout->addRow(setup_button_);
}

void RoboticSystemInputsWidget::perform_initialization(const std::string &tool_file, const std::string &marker_off_file)
{
  tool_off_xform_sel_->setText(QString::fromStdString(tool_file));
  marker_offset_sel_->setText(QString::fromStdString(marker_off_file));
  setup_button_->click();
}

void RoboticSystemInputsWidget::setup_button_clicked()
{
  CMN_LOG_CLASS_RUN_DEBUG << "Setup button clicked!" << std::endl;
  if (tool_off_xform_sel_->isEnabled() && setTransforms())
  {
    enableController();
  }
  else
  {
    disableController();
  }
}

void RoboticSystemInputsWidget::enableController()
{
  CMN_LOG_CLASS_RUN_DEBUG << "Enabling controller!" << std::endl;

  disable_inputs();

  this->SetControllerEnabled(true);

  emit controller_enabled();

}

void RoboticSystemInputsWidget::disableController()
{
  CMN_LOG_CLASS_RUN_DEBUG << "Disabling controller!" << std::endl;
  enable_inputs();
  this->SetControllerEnabled(false);
  emit controller_disabled();
}

bool RoboticSystemInputsWidget::setTransforms()
{
  // vctFrm3 tool_offset_node;
  // bool tool_offset_initialized = this->loadTransformFromSelectionBox(tool_off_xform_sel_, tool_offset_node);
  // if (!tool_offset_initialized)
  // {
  //   CMN_LOG_CLASS_RUN_ERROR << "could not initialize tool offset from file provided" << std::endl;
  //   return false;
  // }
  // this->SetToolOffset(tool_offset_node);

  // vctFrm3 marker_offset_node;
  // bool marker_offset_initialized = this->loadTransformFromSelectionBox(marker_offset_sel_, marker_offset_node);
  // if (!marker_offset_initialized)
  // {
  //   CMN_LOG_CLASS_RUN_ERROR << "could not initialize marker offset from file provided" << std::endl;
  //   return false;
  // }
  // this->SetMarkerOffset(marker_offset_node.Translation());
  
  return true;
}

bool RoboticSystemInputsWidget::loadTransformFromSelectionBox(QLineEdit *sel, vctFrm3 &frame)
{
  std::string file = sel->text().toStdString();
  std::ifstream ifs(file);
  if (!ifs.is_open())
  {
    CMN_LOG_CLASS_RUN_ERROR << "couldn't open transform file " << file << std::endl;
    return false;
  }
  vctMat mat(4, 4, 0.0);
  std::string line;

  for (int i = 0; i < 4; i++)
  {
    getline(ifs, line);
    CMN_LOG_CLASS_RUN_DEBUG << "read line: " << line << std::endl;
    std::istringstream ss(line);

    for (int j = 0; j < 4; j++) {
      double val;
      ss >> val;
      mat(i, j) = val;
    }
    CMN_LOG_CLASS_RUN_DEBUG << "read vals: " << mat(i, 0) << ", " << mat(i, 1) << ", " << mat(i, 2)
      << ", " << mat(i, 3) << std::endl;
  }

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      frame.Rotation()(i, j) = mat(i, j);

  for (int i = 0; i < 3; i++)
    frame.Translation()(i) = mat(i, 3);

  CMN_LOG_CLASS_RUN_DEBUG << "mat = " << std::endl << frame << std::endl;

  return true;
}

void RoboticSystemInputsWidget::enable_inputs()
{
  tool_off_xform_sel_->setEnabled(true);
  marker_offset_sel_->setEnabled(true);
}

void RoboticSystemInputsWidget::disable_inputs()
{
  tool_off_xform_sel_->setEnabled(false);
  marker_offset_sel_->setEnabled(false);
}

