
#include <bigss_ur_gui/RoboticSystemControlWidget.h>

#include <QFormLayout>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QListWidget>

//#include <ctkSliderWidget.h>

#include <bigss_ur_gui/ui_RoboticSystemControlWidget.h>

#include <bigss_robotic_system/mtsSARobSysTask.h>

// #include <bigssMath/slicerUtil.h>

// #include <QtGui>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassServices.h>
#include <cisstCommon/cmnClassRegisterMacros.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsCollectorState.h>

CMN_IMPLEMENT_SERVICES_DERIVED(RoboticSystemControlWidget, mtsComponent);

RoboticSystemControlWidget::RoboticSystemControlWidget(
  const std::string &points_filename,
  const std::string &name,
  QWidget *parent
  ) :
  QWidget(parent),
  mtsComponent(name),
  ui(new Ui::demoControlWidget()),
  collectors()
{
  this->points_filename = points_filename;
  this->setupMultiTask();
  ui->setupUi(this);

  ui->pointListWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
  QPushButton add_current_pos_to_goal_list();
  QPushButton* addCurrentToGoalListButton = new QPushButton(this);
  addCurrentToGoalListButton->setObjectName(QStringLiteral("Add Current To Goal List"));
  addCurrentToGoalListButton->setText("Add Current To Goal List");
  ui->demoPersistentControlsLayout->addWidget(addCurrentToGoalListButton, 0, 0, 1, 2);
  connect(addCurrentToGoalListButton, &QPushButton::released, this, &RoboticSystemControlWidget::on_addCurrentToGoalListButton_clicked);

  QPushButton* disable_button = new QPushButton(this);
  disable_button->setObjectName(QStringLiteral("DISABLE"));
  disable_button->setText("DISABLE");
  ui->demoPersistentControlsLayout->addWidget(disable_button, 1, 0);
  connect(disable_button, &QPushButton::released, this, &RoboticSystemControlWidget::on_disable_button_clicked);
 
  QPushButton* enable_button = new QPushButton(this);
  enable_button->setObjectName(QStringLiteral("ENABLE"));
  enable_button->setText("ENABLE");
  ui->demoPersistentControlsLayout->addWidget(enable_button, 1, 1);
  connect(enable_button, &QPushButton::released, this, &RoboticSystemControlWidget::on_enable_button_clicked);
  }


void RoboticSystemControlWidget::addCollector(mtsManagerLocal *localManager,
  const std::string &taskName, const std::string &stateTableName,
  const std::vector<std::string> &sigs)
{
  // now for all collection
  mtsCollectorState *c = new mtsCollectorState(taskName, stateTableName,
    mtsCollectorBase::COLLECTOR_FILE_FORMAT_CSV);
  int ns = sigs.size();
  for (int i = 0; i < ns; i++)
  {
    c->AddSignal(sigs[i]);
  }

  collectors.push_back(c);

  // add the collector
  localManager->AddComponent(c);
  c->Connect();
  c->Start();
  c->SetOutput("RoboticSystemControlWidget.csv", mtsCollectorBase::COLLECTOR_FILE_FORMAT_CSV);
  //std::cout << "started collector" << std::endl;
}

void RoboticSystemControlWidget::setupMultiTask()
{
  mtsInterfaceRequired *required = this->AddInterfaceRequired("userInterface");
  if (required)
  {
    required->AddFunction("PopGoal", this->PopGoal);
    required->AddFunction("AddGoals", this->AddGoals);
    required->AddFunction("ClearGoal", this->ClearGoal);
    required->AddFunction("MoveGoal", this->MoveGoal);
    required->AddFunction("SetGoal", this->SetGoal);
    required->AddFunction("SetPercentFullSpeed", this->SetPercentFullSpeed);
    required->AddFunction("CartesianPosition", this->GetCartesianPosition);
    required->AddFunction("SetControllerEnabled", this->SetControllerEnabled);
  }

  mtsInterfaceRequired *cuttingInterface = this->AddInterfaceRequired("cuttingInterface");
  if (cuttingInterface)
  {
    cuttingInterface->AddFunction("setDigitalOutput", this->setDigitalOutput);
  }
}

void RoboticSystemControlWidget::on_demoCuttingButton_clicked()
{
  std::string t = ui->demoCuttingButton->text().toStdString();
  if (t.compare("Start Cutting") == 0)
  {
    CMN_LOG_CLASS_RUN_DEBUG << "button clicked, starting cut!" << std::endl;
    this->setDigitalOutput(mtsInt(1));
    ui->demoCuttingButton->setText("Stop Cutting");
  }
  else
  {
    CMN_LOG_CLASS_RUN_DEBUG << "button clicked, stopping cut!" << std::endl;
    this->setDigitalOutput(mtsInt(0));
    ui->demoCuttingButton->setText("Start Cutting");
  }
}

void RoboticSystemControlWidget::on_trajectoryMoveNextButton_clicked()
{
  this->PopGoal();
}

void RoboticSystemControlWidget::on_trajectoryMoveAutoButton_clicked()
{
  size_t n = ui->pointListWidget->count();
  CMN_LOG_CLASS_RUN_DEBUG << "move auto: sending " << n << " points " << std::endl;
  vctMat pointMat(n, 6);
  for (size_t i_pt = 0; i_pt < n; i_pt++)
  {
    QListWidgetItem *item = ui->pointListWidget->item(i_pt);
    vct6 p = stringToPoint(item->text().toStdString()); 
    CMN_LOG_CLASS_RUN_DEBUG << "adding goal: " << item->text().toStdString() << std::endl;
    for(size_t j=0; j<6; j++){
      pointMat(i_pt,j) = p(j);
    }
  }
  this->AddGoals(pointMat); 
}

// Clear the points from the Point list
void RoboticSystemControlWidget::on_clearPointsButton_clicked(){
  ui->pointListWidget->clear();
}

// Reset the robot to the original position it was in when it started
void RoboticSystemControlWidget::on_resetToInitPosButton_clicked(){

}

void RoboticSystemControlWidget::on_addCurrentToGoalListButton_clicked(){
  this->GetCartesianPosition(cartesian_position);
  QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(cartesian_position.ToString()));
  item->setFlags(item->flags() | Qt::ItemIsEditable);
  ui->pointListWidget->addItem(item);

}

void RoboticSystemControlWidget::on_disable_button_clicked(){
  this->SetControllerEnabled(false);
}

void RoboticSystemControlWidget::on_enable_button_clicked(){
  this->SetControllerEnabled(true);
}

void RoboticSystemControlWidget::on_loadPointsButton_clicked()
{
  std::ifstream ifs(this->points_filename);
  if (!ifs.is_open()) {
    CMN_LOG_CLASS_RUN_ERROR << "error opening: " + this->points_filename << std::endl;
    return;
  }

  std::string line;

  while(getline(ifs, line))
  {
    QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(line));
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    ui->pointListWidget->addItem(item);
  }


/*  ui->pointListWidget->clear();

  fiducials points_fids = read_slicer_fiducials("trajectory.fcsv", false);
  std::cout << points_fids << std::endl;
  int n = points_fids.points.cols();
  for (int idx = 0; idx < n; idx++)
  {
    std::string pt_str =
      std::to_string(points_fids.points(0, idx)) + ", "
      + std::to_string(points_fids.points(1, idx)) + ", "
      + std::to_string(points_fids.points(2, idx));
    QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(pt_str));
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    ui->pointListWidget->addItem(item);
  }
*/
// HTP change>


}

/*
void RoboticSystemControlWidget::readPointsFromTextFile()
{
  std::ifstream ifs("points.txt");
  if (!ifs.is_open()) {
    CMN_LOG_CLASS_RUN_ERROR << "error opening points.txt!" << std::endl;
    return;
  }

  std::string line;

  while(getline(ifs, line))
  {
    QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(line));
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    ui->pointListWidget->addItem(item);
  }
}
*/


void RoboticSystemControlWidget::on_savePointsButton_clicked()
{
  std::ofstream ofs(this->points_filename);
  if (!ofs.is_open()) {
    CMN_LOG_CLASS_RUN_ERROR << "error opening: " + this->points_filename << std::endl;
    return;
  }

  std::string line;

  int numItems = ui->pointListWidget->count();
  for(int i = 0; i < numItems; i++)
  {
    std::string pointString = ui->pointListWidget->item(i)->text().toStdString();
    ofs << pointString << std::endl;
  }

  ofs.close();
}

void RoboticSystemControlWidget::setCurrentListItemToPoint(const vct6 &v)
{
  QListWidgetItem *current = ui->pointListWidget->currentItem();
  if (!current)
  {
    CMN_LOG_RUN_ERROR << "no point selected!" << std::endl;
    return;
  }
  pointToListItem(v, current);
}

bool RoboticSystemControlWidget::listItemToPoint(QListWidgetItem *item, vct6 &p)
{
  if(!item)
  {
    CMN_LOG_RUN_ERROR << "no point selected!" << std::endl;
    return false;
  }
  p = stringToPoint(item->text().toStdString()); 
  return true;
}

void RoboticSystemControlWidget::pointToListItem(const vct6 &p, QListWidgetItem *item)
{
  std::string pt_str = pointToStdString(p);
  item->setText(QString::fromStdString(pt_str));
}

std::string RoboticSystemControlWidget::pointToStdString(const vct6 &p)
{
  std::string out_str;
  for(auto& x: p){
    out_str += std::to_string(x);
  }  
  return out_str;
}

vct6 RoboticSystemControlWidget::stringToPoint(const std::string &s)
{
  vct6 v;
  std::istringstream ss(s);
  char c;

  for (int i = 0; i < 6; i++) {
    double val;
    ss >> val;
    ss.get(c);
    v(i) = val;
  }
  return v;
}

void RoboticSystemControlWidget::on_goalPointNegativeXButton_clicked()
{
  double d = ui->goalPointStepBox->value();
  this->MoveGoal(vct6(-d, 0.0, 0.0, 0.0, 0.0, 0.0));
}

void RoboticSystemControlWidget::on_goalPointNegativeYButton_clicked()
{
  double d = ui->goalPointStepBox->value();
  this->MoveGoal(vct6(0.0, -d, 0.0, 0.0, 0.0, 0.0));
}

void RoboticSystemControlWidget::on_goalPointNegativeZButton_clicked()
{
  double d = ui->goalPointStepBox->value();
  this->MoveGoal(vct6(0.0, 0.0, -d, 0.0, 0.0, 0.0));
}

void RoboticSystemControlWidget::on_goalPointPositiveXButton_clicked()
{
  double d = ui->goalPointStepBox->value();
  this->MoveGoal(vct6(d, 0.0, 0.0, 0.0, 0.0, 0.0));
}

void RoboticSystemControlWidget::on_goalPointPositiveYButton_clicked()
{
  double d = ui->goalPointStepBox->value();
  this->MoveGoal(vct6(0.0, d, 0.0, 0.0, 0.0, 0.0));
}

void RoboticSystemControlWidget::on_goalPointPositiveZButton_clicked()
{
  double d = ui->goalPointStepBox->value();
  this->MoveGoal(vct6(0.0, 0.0, d, 0.0, 0.0, 0.0));
}

void RoboticSystemControlWidget::on_demoSpeedSlider_valueChanged(int d)
{
  double percent_full_speed = d/100.0;
  this->SetPercentFullSpeed(percent_full_speed);
  std::cout << "PercentFullSpeed set to:" << percent_full_speed << std::endl;
}

void RoboticSystemControlWidget::on_goalPointGoButton_clicked()
{
  QList<QListWidgetItem*> goalList = ui->pointListWidget->selectedItems();
  int num_goalpoints = goalList.count();
  CMN_LOG_CLASS_RUN_DEBUG << "sending " << num_goalpoints << " points" << std::endl;

  vctMat pointMat(num_goalpoints, 6);

  for (int i = 0; i < num_goalpoints; i++)
  {
    QListWidgetItem *item = goalList.at(i);
    vct6 p = stringToPoint(item->text().toStdString()); 
    CMN_LOG_CLASS_RUN_DEBUG << "adding goal: " << item->text().toStdString() << std::endl;
    pointMat(i, 0) = p(0);
    pointMat(i, 1) = p(1);
    pointMat(i, 2) = p(2);
    pointMat(i, 3) = p(3);
    pointMat(i, 4) = p(4);
    pointMat(i, 5) = p(5);

  }

  this->AddGoals(pointMat); 
}

void RoboticSystemControlWidget::on_goalPointStopButton_clicked()
{
  CMN_LOG_CLASS_RUN_DEBUG << "sending clear goal from control widget" << std::endl;
  this->ClearGoal();
}

void RoboticSystemControlWidget::on_resetGoalPointButton_clicked()
{
  CMN_LOG_CLASS_RUN_ERROR << "this button does nothing right now" << std::endl;
}

void RoboticSystemControlWidget::on_loggingBox_stateChanged()
{
  if (ui->loggingBox->isChecked())
  {
    CMN_LOG_CLASS_RUN_DEBUG << "starting collectors" << std::endl;
    this->startCollectors();
  }
  else
  {
    CMN_LOG_CLASS_RUN_DEBUG << "stopping collectors" << std::endl;
    this->stopCollectors();
  }
}

void RoboticSystemControlWidget::startCollectors()
{
  int n = this->collectors.size();
  for (int i = 0; i < n; i++)
    collectors[i]->StartCollection(0.0);
}

void RoboticSystemControlWidget::stopCollectors()
{
  int n = this->collectors.size();
  for (int i = 0; i < n; i++)
    collectors[i]->StopCollection(0.0);
}

