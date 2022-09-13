#include <ur_cb2_gui/RoboticSystemLoggingWidget.h>

#include <QGridLayout>
#include <QPushButton>
#include <QTimer>

//#include <ctkPathLineEdit.h>
// #include <qMRMLNodeComboBox.h>

#include <ur_cb2_gui/ui_RoboticSystemLoggingWidget.h>

// #include <QtGui>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassServices.h>
#include <cisstCommon/cmnClassRegisterMacros.h>

CMN_IMPLEMENT_SERVICES_DERIVED(RoboticSystemLoggingWidget, mtsComponent);

RoboticSystemLoggingWidget::RoboticSystemLoggingWidget(
    const std::string &name, QWidget *parent)
  :
  QWidget(parent),
  mtsComponent(name),
  recordInterval(1.0), // s
  recordTimer(new QTimer(this)),
  ui(new Ui::demoLoggingWidget())
{
  recordTimer->setObjectName("recordTimer");

  ui->setupUi(this);

  qMRMLNodeComboBox *selectionBox = ui->loggingFiducialsSelectionBox;
  selectionBox->setNodeTypes(QStringList() << "vtkMRMLMarkupsFiducialNode" << "vtkMRMLModelNode");
  selectionBox->setAddEnabled(true);
  selectionBox->setRenameEnabled(true);

  ui->loggingAutoRecordButton->setCheckable(true);
  ui->loggingAutoRecordButton->setChecked(false);
/*
  connect(recordTimer, SIGNAL(timeout()), this, SLOT(on_recordTimer_timeout())); 

  connect(ui->loggingFiducialsSelectionBox, SIGNAL(currentNodeChanged(vtkMRMLNode*)),
    this, SLOT(on_loggingFiducialsSelectionBox_currentNodeChanged(vtkMRMLNode*)));

  connect(ui->loggingRecordButton, SIGNAL(clicked()), this,
    SLOT(on_loggingRecordButton_clicked()));

  connect(ui->loggingRecordIntervalBox, SIGNAL(valueChanged(double)), this,
    SLOT(on_loggingRecordIntervalBox_valueChanged(double)));

  connect(ui->loggingAutoRecordButton, SIGNAL(clicked(bool)), this,
    SLOT(on_loggingAutoRecordButton_clicked(bool)));

  connect(ui->loggingClearPointsButton, SIGNAL(clicked()), this,
    SLOT(on_loggingClearPointsButton_clicked(bool))); */
}

void RoboticSystemLoggingWidget::updateSelections(vtkMRMLScene *scene)
{
  ui->loggingFiducialsSelectionBox->setMRMLScene(scene);
}

void RoboticSystemLoggingWidget::on_loggingFiducialsSelectionBox_currentNodeChanged(vtkMRMLNode *node)
{
//  logic_->setLoggingNode(node);
}

void RoboticSystemLoggingWidget::on_loggingRecordButton_clicked()
{
//  logic_->recordPoint();
}

void RoboticSystemLoggingWidget::on_loggingRecordIntervalBox_valueChanged(double d)
{
  recordInterval = d;
}

void RoboticSystemLoggingWidget::on_loggingAutoRecordButton_toggled(bool checked)
{
  if (checked)
  {
    ui->loggingAutoRecordButton->setText("Stop Recording");
    recordTimer->start(recordInterval*1000);
  }
  else
  {
    ui->loggingAutoRecordButton->setText("Auto Record");
    recordTimer->stop();
  }
}

void RoboticSystemLoggingWidget::on_loggingClearPointsButton_clicked()
{
//  logic_->clearPoints();
}

void RoboticSystemLoggingWidget::on_recordTimer_timeout()
{
//  logic_->recordPoint();
}

