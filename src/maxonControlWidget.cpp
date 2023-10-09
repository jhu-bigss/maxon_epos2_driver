#include "maxon_epos2_driver/maxonControlWidget.h"

#include <QTimer>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QButtonGroup>
#include <QRadioButton>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QString>
#include <QMessageBox>
#include <QTabWidget>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsManagerLocal.h>

CMN_IMPLEMENT_SERVICES_DERIVED(maxonControlWidget, mtsComponent)

maxonControlWidget::maxonControlWidget(const std::string &taskName, QWidget *parent) :
  QWidget(parent),
  mtsComponent(taskName),
  useRaw(true)
{
  CMN_LOG_CLASS_INIT_DEBUG << "constructing control widget" << std::endl;

  // interfaces
  mtsInterfaceRequired *required = AddInterfaceRequired("control");
  if(required) {
    required->AddFunction("enable", maxon.enable);
    required->AddFunction("disable", maxon.disable);
    required->AddFunction("clearFault", maxon.clearFault);
    required->AddFunction("stop", maxon.stop);
    required->AddFunction("setVelocity", maxon.setVelocity);
    required->AddFunction("setVelocity_raw", maxon.setVelocity_raw);
    required->AddFunction("setVelocityPoint", maxon.setVelocityPoint);
    required->AddFunction("setVelocityPoint_raw", maxon.setVelocityPoint_raw);
    required->AddFunction("getTargetVelocity", maxon.getTargetVelocity);
    required->AddFunction("getTargetVelocity_raw", maxon.getTargetVelocity_raw);
    required->AddFunction("jogPlus", maxon.jogPlus);
    required->AddFunction("jogMinus", maxon.jogMinus);
    required->AddFunction("moveToAbsolutePosition", maxon.moveToAbsolutePosition);
    required->AddFunction("moveToRelativePosition", maxon.moveToRelativePosition);
    required->AddFunction("moveToAbsolutePosition_raw", maxon.moveToAbsolutePosition_raw);
    required->AddFunction("moveToRelativePosition_raw", maxon.moveToRelativePosition_raw);
    required->AddFunction("setPositionMode", maxon.setPositionMode);
    required->AddFunction("setVelocityMode", maxon.setVelocityMode);
    required->AddFunction("setSetPointMode", maxon.setSetPointMode);
    required->AddFunction("getErrorString", maxon.getErrorString);
    required->AddFunction("getNodeId", maxon.getNodeId);
    required->AddFunction("zero", maxon.zero);
    required->AddFunction("moveToZero", maxon.moveToZero);
    required->AddFunction("rebias", maxon.rebias);
    required->AddFunction("setHomeForce", maxon.setHomeForce);
    required->AddFunction("startForceHoming", maxon.startForceHoming);

    required->AddFunction("negativeLimit", maxon.negativeLimit);
    required->AddFunction("positiveLimit", maxon.positiveLimit);
    required->AddFunction("setNegativeLimit", maxon.setNegativeLimit);
    //required->AddFunction("setPositiveLimit", maxon.setPositiveLimit);
    required->AddFunction("setNegativeLimitCurrentPosition", maxon.setNegativeLimitCurrentPosition);
    //required->AddFunction("setPositiveLimitCurrentPosition", maxon.setPositiveLimitCurrentPosition);
  }

  // required = AddInterfaceRequired("state");
  // if(required) {
  //   required->AddFunction("force", maxon.getCableForce);
  // }

  CMN_LOG_CLASS_INIT_DEBUG << "setting up ui" << std::endl;
  setupUi();
  QMetaObject::connectSlotsByName(this);
}

maxonControlWidget::~maxonControlWidget()
{

}

void maxonControlWidget::setupUi()
{
  QVBoxLayout *layout = new QVBoxLayout(this);

  ////////////
  // Enable //
  ////////////
  QPushButton *enabled = new QPushButton(this);
  enabled->setText("Enable");
  enabled->setObjectName("enable");
  enabled->setCheckable(true);

  layout->addWidget(enabled);

  QPushButton *clearFault = new QPushButton(this);
  clearFault->setText("Clear Fault");
  clearFault->setObjectName("clearFault");

  layout->addWidget(clearFault);

  ////////////
  // Limits //
  ////////////
  QWidget *limits = new QWidget(this);
  QHBoxLayout *limitsLayout = new QHBoxLayout(limits);
  QPushButton *negLimit = new QPushButton(limits);
  negLimit->setText("Set negative limit");
  negLimit->setObjectName("negLimit");
  //QPushButton *posLimit = new QPushButton(limits);
  //posLimit->setText("Set positive limit");
  //posLimit->setObjectName("posLimit");
  limitsLayout->addWidget(negLimit);
  //limitsLayout->addWidget(posLimit);
  limits->setLayout(limitsLayout);

  layout->addWidget(limits);

  ////////////////////////
  // Universal Controls //
  ////////////////////////
  QGroupBox *params = new QGroupBox(this);
  QFormLayout *paramsLayout = new QFormLayout(params);

  QCheckBox *unitDisplay = new QCheckBox(this);
  unitDisplay->setText("Display units");
  unitDisplay->setObjectName("displayUnits");
  paramsLayout->addWidget(unitDisplay);

  vel = new QDoubleSpinBox(this);
  vel->setObjectName("velocity");
  vel->setSuffix(" rpm");
  vel->setRange(0, 15000);
  vel->setSingleStep(100);
  paramsLayout->addRow("Velocity", vel);

  QPushButton *stop = new QPushButton("Stop", this);
  stop->setObjectName("stop");
  paramsLayout->addWidget(stop);  

  params->setLayout(paramsLayout);
  layout->addWidget(params);

  ///////////////////
  // Control Modes //
  ///////////////////
  QTabWidget *opModeTabWidget = new QTabWidget();
  opModeTabWidget->setObjectName("opModeTabWidget");
  layout->addWidget(opModeTabWidget);

  ///////////////////////
  // Position movement //
  ///////////////////////
  posGroup = new QGroupBox(this);
  posGroup->setTitle("Position");
  QGridLayout *posLayout = new QGridLayout(posGroup);
  posGroup->setLayout(posLayout);

  absoluteMove = new QCheckBox(posGroup);
  absoluteMove->setText("Absolute movement");
  posLayout->addWidget(absoluteMove, 0, 0, 1, 2);

  position = new QDoubleSpinBox(posGroup);
  position->setRange(-30000, 30000);
  position->setSingleStep(1000);
  position->setSuffix("enc");
  posLayout->addWidget(position, 1, 1);

  QPushButton *posMove = new QPushButton("Move");
  posMove->setObjectName("movePosition");
  posLayout->addWidget(posMove, 2, 0, 1, 2);

  QPushButton *zeroMove = new QPushButton("Move to zero");
  zeroMove->setObjectName("moveToZero");
  posLayout->addWidget(zeroMove, 3, 0, 1, 2);

  QPushButton *zero = new QPushButton();
  zero->setText("Zero position");
  zero->setObjectName("zero");
  posLayout->addWidget(zero);

  posLayout->addWidget(new QLabel("Position"), 1, 0);
  opModeTabWidget->insertTab(0, posGroup, QString::fromStdString("Position"));

  ///////////////////////
  // Velocity movement //
  ///////////////////////
  velGroup = new QGroupBox(this);
  QHBoxLayout *velLayout = new QHBoxLayout(velGroup);

  QPushButton *moveVelPlus = new QPushButton("Jog +", velGroup);
  moveVelPlus->setObjectName("jogPlus");
  QPushButton *moveVelMinus = new QPushButton("Jog -", velGroup);
  moveVelMinus->setObjectName("jogMinus");

  velLayout->addWidget(moveVelMinus);
  velLayout->addWidget(moveVelPlus);
  velGroup->setLayout(velLayout);
  velGroup->setEnabled(false);

  opModeTabWidget->insertTab(1, velGroup, QString::fromStdString("Velocity"));

  ////////////////////////
  // Set point movement //
  ////////////////////////
  velocityPointBox = new QDoubleSpinBox();
  velocityPointBox->setObjectName("velocityPointBox");
  velocityPointBox->setMinimum(-2.0);
  velocityPointBox->setMaximum(2.0);
  velocityPointBox->setSingleStep(0.1);
  velocityPointBox->setSuffix(" rpm");
  opModeTabWidget->insertTab(2, velocityPointBox, QString::fromStdString("Set Point"));

  ////////////
  // Homing //
  ////////////
  QGroupBox *homingGroup = new QGroupBox(this);
  QFormLayout *homingLayout = new QFormLayout(homingGroup);
  homingGroup->setTitle("Homing");
  
  homingForce = new QDoubleSpinBox(this);
  homingForce->setObjectName("homeForce");
  homingForce->setSuffix(" N");
  homingForce->setValue(0.2);
  homingForce->setRange(0, 0.5);
  homingForce->setSingleStep(0.05);
  homingLayout->addRow("Homing Force", homingForce);

  QPushButton *rebias = new QPushButton(this);
  rebias->setText("Rebias");
  rebias->setObjectName("rebias");
  homingLayout->addRow(rebias);

  QPushButton *home = new QPushButton(this);
  home->setText("Home");
  home->setObjectName("home");
  homingLayout->addRow(home);

  homingGroup->setLayout(homingLayout);
  layout->addWidget(homingGroup);

  ////////////
  // Layout //
  ////////////
  layout->addStretch(1);
  this->setLayout(layout);
}

///////////////
// Callbacks //
///////////////
void maxonControlWidget::on_enable_toggled(bool checked)
{
  if(checked) {
    mtsBool enabled;
    maxon.enable(enabled);
    if(!enabled) {
      mtsStdString err;
      maxon.getErrorString(err);

      mtsUShort nodeId;
      maxon.getNodeId(nodeId);
      QMessageBox::warning(this, "Maxon", "Could not enable motor node " + QString::number(nodeId.GetData()) + "\n" + QString::fromStdString(err.Data));
      dynamic_cast<QPushButton *>(QObject::sender())->setChecked(false);
    }
  } else {
    maxon.disable();
  }
}

void maxonControlWidget::on_rebias_clicked()
{
  maxon.rebias();
}

void maxonControlWidget::on_homeForce_valueChanged(double val) 
{
  maxon.setHomeForce(val);
}

void maxonControlWidget::on_home_clicked()
{
  maxon.startForceHoming();
}

void maxonControlWidget::on_clearFault_clicked()
{
  maxon.clearFault();
}

void maxonControlWidget::on_zero_clicked()
{
  maxon.zero();
}

void maxonControlWidget::on_displayUnits_clicked(bool checked)
{
  useRaw = !checked;
  vel->setValue(0.0);

  // ToDo: Make these non-arbitrary numbers
  if(useRaw) {
    mtsLong velocity;
    maxon.getTargetVelocity_raw(velocity);
    vel->setRange(0.0, 20000.0);
    vel->setSingleStep(1000.0);
    vel->setSuffix(" rpm");
    vel->setValue(velocity.GetData());
    position->setRange(-30000, 30000);
    position->setSingleStep(1000);
    position->setSuffix(" enc");
    velocityPointBox->setSuffix(" rpm");
  } else {
    mtsDouble velocity;
    maxon.getTargetVelocity(velocity);
    vel->setRange(0.0, 10.0);
    vel->setSingleStep(0.5);
    vel->setSuffix(" mm/s");
    vel->setValue(velocity.GetData());
    position->setRange(-20.0, 20.0);
    position->setSingleStep(0.5);
    position->setSuffix(" mm");
    velocityPointBox->setSuffix(" mm/s");
  }
}

void maxonControlWidget::on_velocity_valueChanged(double val)
{
  if(useRaw) {
    mtsLong vel = (long)val;
    maxon.setVelocity_raw(vel);
  }
  else {
    maxon.setVelocity(val);
  }
}

void maxonControlWidget::on_velocityPointBox_valueChanged(double val)
{
  CMN_LOG_CLASS_RUN_DEBUG << "velocity point value changed!" << std::endl;
  if(useRaw) {
    maxon.setVelocityPoint_raw(val);
  }
  else {
    maxon.setVelocityPoint(val);
  }
}

void maxonControlWidget::on_jogPlus_clicked()
{
  CMN_LOG_CLASS_RUN_DEBUG << "jog plus!" << std::endl;
  maxon.jogPlus();
}

void maxonControlWidget::on_jogMinus_clicked()
{
  CMN_LOG_CLASS_RUN_DEBUG << "jog minus!" << std::endl;
  maxon.jogMinus();
}

void maxonControlWidget::on_opModeTabWidget_currentChanged(int id)
{
  CMN_LOG_CLASS_RUN_DEBUG << "changing operating mode " << id << std::endl;
  maxon.stop();
  maxon.setSetPointMode(false);
  switch(id) {
    case 0:
      maxon.setPositionMode();
      velGroup->setEnabled(false);
      posGroup->setEnabled(true);
      break;
    case 1:
      maxon.setVelocityMode();
      velGroup->setEnabled(true);
      posGroup->setEnabled(false);
      break;
    case 2:
      maxon.setSetPointMode(true);
      maxon.setVelocityMode();
      velGroup->setEnabled(true);
      posGroup->setEnabled(false);
  }
}

void maxonControlWidget::on_movePosition_clicked()
{
  double pos = position->value();
  bool absolute = absoluteMove->isChecked();

  if(useRaw) {
    mtsLong lpos = pos;
    if(absolute) 
      maxon.moveToAbsolutePosition_raw(lpos);
    else
      maxon.moveToRelativePosition_raw(lpos);
  }
  else {
    if(absolute)
      maxon.moveToAbsolutePosition(pos);
    else
      maxon.moveToRelativePosition(pos);
  }
}

void maxonControlWidget::on_moveToZero_clicked()
{
  maxon.moveToZero();
}

void maxonControlWidget::on_stop_clicked()
{
  maxon.stop();
}

void maxonControlWidget::on_negLimit_clicked()
{
  maxon.setNegativeLimitCurrentPosition();
}

//void maxonControlWidget::on_posLimit_clicked()
//{
//  maxon.setPositiveLimitCurrentPosition();
//}
