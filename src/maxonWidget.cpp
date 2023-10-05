#include "maxon_epos2_driver/maxonWidget.h"
#include "maxon_epos2_driver/maxonStatusWidget.h" 
#include "maxon_epos2_driver/maxonControlWidget.h"

#include "maxon_epos2_driver/maxonInterface.h"
#include <cisstMultiTask/mtsManagerLocal.h>

#include <QTimer>
#include <QComboBox>
#include <QCheckBox>
#include <QVBoxLayout>

CMN_IMPLEMENT_SERVICES(maxonWidget)

maxonWidget::maxonWidget(maxonInterface *maxon_, QWidget *parent) :
  QWidget(parent),
  maxon(maxon_),
  statusWidget(new maxonStatusWidget(maxon->getMotorList())),
  timer(new QTimer(this)),
  motorIndex(0)
{
  CMN_LOG_CLASS_INIT_DEBUG << "setting up user interface" << std::endl;
  setupUi();

  // setup the timer
  CMN_LOG_CLASS_INIT_DEBUG << "setting timer name" << std::endl;
  timer->setObjectName("timer");

  CMN_LOG_CLASS_INIT_DEBUG << "connecting qt slots for maxon widget" << std::endl;
  QMetaObject::connectSlotsByName(this);

  CMN_LOG_CLASS_INIT_DEBUG << "starting qt timer" << std::endl;
  timer->start(20.0);
}

void maxonWidget::Connect(mtsManagerLocal *localManager)
{
  statusWidget->Connect(localManager);
  for(int i = 0; i < controls.size(); i++)
  {
    std::string name = maxon->getMotor(i)->GetName();;
    localManager->Connect(this->controls[i]->GetName(), "control",  name, "control");
  }
}

maxonWidget::~maxonWidget()
{
  timer->stop();
}

void maxonWidget::setupUi()
{
  QVBoxLayout *layout = new QVBoxLayout(this);

  QComboBox *cmb = new QComboBox(this);
  cmb->setObjectName("selectNode");

  QCheckBox *cbk = new QCheckBox(this);
  cbk->setObjectName("recordData");
  cbk->setText("Record data");

  layout->addWidget(cmb);
  layout->addWidget(cbk);
  layout->addWidget(statusWidget);

  for(unsigned int i=0; i<maxon->getNumMotors(); i++) {
    cmb->addItem(QString("Node ") + QString::number(i));

    CMN_LOG_CLASS_INIT_DEBUG << "creating control widget" << std::endl;
    maxonControlWidget *control = new maxonControlWidget(
      maxon->getMotor(i)->GetName() + "ControlWidget", this);
    layout->addWidget(control);
    control->setVisible(false);
    controls.push_back(control);
  }

  if (controls.size() > 0)
    controls[0]->setVisible(true);
  CMN_LOG_CLASS_INIT_DEBUG << "setting layout" << std::endl;
  this->setLayout(layout);
}

///////////////
// Callbacks //
///////////////
void maxonWidget::on_selectNode_currentIndexChanged(int index)
{
  for(int i=0; i<controls.size(); i++) {
    controls[i]->setVisible(false);
  }

  controls[index]->setVisible(true);
  motorIndex = index;
}

void maxonWidget::on_recordData_toggled(bool checked)
{
  if (checked)
    maxon->startCollection();
  else
    maxon->stopCollection();
}

void maxonWidget::on_timer_timeout()
{
  if(statusWidget->update(motorIndex)) {
    statusWidget->setEnabled(true);
  }
  else {
    statusWidget->setEnabled(false);
  }
}

