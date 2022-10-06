#include <bigss_ur_gui/RoboticSystemWidget.h>

#include <bigss_ur_gui/RoboticSystemInputsWidget.h>
#include <bigss_ur_gui/RoboticSystemControlWidget.h>
#include <bigss_ur_gui/RoboticSystemConstraintsWidget.h>
#include <bigss_ur_gui/RoboticSystemStatusWidget.h>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QTabWidget>

#include <cisstCommon/cmnQt.h>
//#include <ctkCollapsibleButton.h>


RoboticSystemWidget::~RoboticSystemWidget()
{
}

RoboticSystemWidget::RoboticSystemWidget(std::string points_filename, int num_joints, QWidget *parent)
{

  if(!num_joints){
    std::cout << "Warning: GUI parameters set for system with zero joints, you may need to initialize if not intended" << std::endl;
  }

  // some things for this widget
  if (parent)
    this->setParent(parent);
  
  QGridLayout *layout = new QGridLayout(this);
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(on_timer_timeout()));

  tabs_ = new QTabWidget(this);

  // setup the inputs widget
  inputs_ = new RoboticSystemInputsWidget();
  //tabs_->addTab(inputs_, "Inputs"); 
  //inputs_button_ = wrapInDropdownButton("Inputs", inputs_);
  layout->addWidget(inputs_, 0, 0);

  connect(inputs_, SIGNAL(controller_enabled()), this,
      SLOT(start_controller()));
  connect(inputs_, SIGNAL(controller_disabled()), this,
      SLOT(stop_controller()));

  // setup the status widget
  status_ = new RoboticSystemStatusWidget("StatusWidget",0,num_joints);
  status_->setEnabled(true);
  //tabs_->addTab(status_, "Status");


  //status_button_ = wrapInDropdownButton("Status", status_);
  //htp_to_ur_only//status_button_->setCollapsed(true);
  layout->addWidget(status_, 0, 1);

  // setup the control widget
  control_ =
      new RoboticSystemControlWidget(points_filename);
  control_->setEnabled(true);
  //tabs_->addTab(control_, "Control");
  //control_button_ = wrapInDropdownButton("Control", control_);
  //htp_to_ur_only//control_button_->setCollapsed(true);
  layout->addWidget(control_, 1, 0);

  // setup the constraints widget
  constraints_ = new RoboticSystemConstraintsWidget(num_joints);
  constraints_->setEnabled(true);
  //tabs_->addTab(constraints_, "Constraints");

  //constraints_button_ = wrapInDropdownButton("Constraints", constraints_);
  //htp_to_ur_only// constraints_button_->setCollapsed(true);
  layout->addWidget(constraints_, 1, 1);
  //layout->addWidget(tabs_,0,0);
  // setup the logging widget
  //  logging_ =
  //    new RoboticSystemLoggingWidget();
  //  logging_->setEnabled(false);
  //  logging_button_ = wrapInDropdownButton("Logging", logging_);
  //  logging_button_->setCollapsed(true);
  //  layout->addWidget(logging_button_);
  //  connect(this->parentWidget(), SIGNAL(mrmlSceneChanged(vtkMRMLScene*)),
  //    logging_, SLOT(updateSelections(vtkMRMLScene*)));

  this->setLayout(layout);
}

// QTabWidget* RoboticSystemWidget::wrapInDropdownButton(
//     const std::string &title, QWidget *widget)
// {
//   QTabWidget *tab = new QTabWidget(QString::fromStdString(title));
//   QVBoxLayout *layout = new QVBoxLayout();
//   layout->addWidget(widget);
//   tab->setLayout(layout);
//   return tab;
// }

void RoboticSystemWidget::start_controller()
{
  timer_->start(100);
  this->enable_controller_buttons();
}

void RoboticSystemWidget::stop_controller()
{
  this->disable_controller_buttons();
  timer_->stop();
}

void RoboticSystemWidget::on_timer_timeout()
{
  status_->update();
}

void RoboticSystemWidget::disable_controller_buttons()
{
  status_->setEnabled(false);
  control_->setEnabled(false);
  constraints_->setEnabled(false);
  // logging_->setEnabled(false);

 //htp_to_ur_only// status_button_->setCollapsed(true);
 //htp_to_ur_only// control_button_->setCollapsed(true);
 //htp_to_ur_only// constraints_button_->setCollapsed(true);
  // logging_button_->setCollapsed(true);
}

void RoboticSystemWidget::enable_controller_buttons()
{
  status_->setEnabled(true);
  control_->setEnabled(true);
  constraints_->setEnabled(true);
  // logging_->setEnabled(true);

  //htp_to_ur_only//status_button_->setCollapsed(false);
  //htp_to_ur_only//control_button_->setCollapsed(false);
  //htp_to_ur_only//constraints_button_->setCollapsed(false);
  // logging_button_->setCollapsed(false);
}


