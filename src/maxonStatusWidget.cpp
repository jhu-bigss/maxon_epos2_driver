#include "maxon_epos2_driver/maxonStatusWidget.h"

#include <QFormLayout>
#include <QLineEdit>
#include <QString>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>

CMN_IMPLEMENT_SERVICES_DERIVED(maxonStatusWidget, mtsComponent)

maxonStatusWidget::maxonStatusWidget(const std::vector<std::string> &names, const std::string &taskName, QWidget *parent) :
  QWidget(parent),
  mtsComponent(taskName),
  maxonInterface(),
  motorList(names)
{
  setupUi();
  setupMultiTask();
}

maxonStatusWidget::~maxonStatusWidget()
{

}

void maxonStatusWidget::Connect(mtsManagerLocal *localManager)
{
  for (int i = 0; i < motorList.size(); i++)
  {
    std::cout << "motorList[i] = " << motorList[i] <<std::endl;
    localManager->Connect(this->GetName(), motorList[i] + "StateInterface", motorList[i], "state");
  }
}

bool maxonStatusWidget::update(int i)
{
  if(enabled)
    enabled->setText("True");
  else
    enabled->setText("False");

  std::string opModeVal;
  mtsExecutionResult res = maxonInterface[i]->operationMode(opModeVal);

  if (!res)
    return false;

  opMode->setText(QString::fromStdString(opModeVal));

  double positionVal;
  res = maxonInterface[i]->position(positionVal);

  if (!res)
    return false;

  position->setText(QString::number(positionVal));

  double velocityVal;
  res = maxonInterface[i]->velocity(velocityVal);

  if (!res)
    return false;

  velocity->setText(QString::number(velocityVal));

  double velocityAverageVal;
  res = maxonInterface[i]->velocityAverage(velocityAverageVal);

  if (!res)
    return false;

  velocityAveraged->setText(QString::number(velocityAverageVal));

#if MAXON_DISPLAY_RAW
  positionRaw->setText(QString::number(status.position_raw));
  velocityRaw->setText(QString::number(status.velocity_raw));
  velocityAveragedRaw->setText(QString::number(status.velocityAverage_raw));
#endif

  short currentVal;
  res = maxonInterface[i]->current(currentVal);

  if (!res)
    return false;

  current->setText(QString::number(currentVal));

  short currentAverageVal;
  res = maxonInterface[i]->current(currentAverageVal);

  if (!res)
    return false;

  currentAveraged->setText(QString::number(static_cast<int>(currentAverageVal)));

  double forceVal;
  res = maxonInterface[i]->force(forceVal);

  if (!res)
    return false;

  force->setText(QString::number(forceVal));

  return true;
}

void maxonStatusWidget::setupUi()
{
  QFormLayout *layout = new QFormLayout(this);

  enabled = new QLineEdit(this);
  enabled->setText("False");
  enabled->setReadOnly(true);

  opMode = new QLineEdit(this);
  opMode->setText("None");
  opMode->setReadOnly(true);

  position = new QLineEdit(this);
  position->setText("0");
  position->setReadOnly(true);

  velocity = new QLineEdit(this);
  velocity->setText("0");
  velocity->setReadOnly(true);

  velocityAveraged = new QLineEdit(this);
  velocityAveraged->setText("0");
  velocityAveraged->setReadOnly(true);

#if MAXON_DISPLAY_RAW
  positionRaw = new QLineEdit(this);
  positionRaw->setText("0");
  positionRaw->setReadOnly(true);

  velocityRaw = new QLineEdit(this);
  velocityRaw->setText("0");
  velocityRaw->setReadOnly(true);

  velocityAveragedRaw = new QLineEdit(this);
  velocityAveragedRaw->setText("0");
  velocityAveragedRaw->setReadOnly(true);
#endif

  current = new QLineEdit(this);
  current->setText("0");
  current->setReadOnly(true);

  currentAveraged = new QLineEdit(this);
  current->setText("0");
  current->setReadOnly(true);

  force = new QLineEdit(this);
  force->setText("0");
  force->setReadOnly(true);

  layout->addRow("Enabled", enabled);  
  layout->addRow("Operation mode", opMode);
  layout->addRow("Position", position);
#if MAXON_DISPLAY_RAW
  layout->addRow("Position [enc]", positionRaw);
#endif
  layout->addRow("Velocity", velocity);
#if MAXON_DISPLAY_RAW
  layout->addRow("Velocity [rpm]", velocityRaw);
#endif
  layout->addRow("Average velocity", velocityAveraged);
#if MAXON_DISPLAY_RAW
  layout->addRow("Average velocity [rpm]", velocityAveragedRaw);
#endif
  layout->addRow("Current", current);
  layout->addRow("Average current", currentAveraged);
  layout->addRow("Force [lbs]", force);

  this->setLayout(layout);
}

void maxonStatusWidget::setupMultiTask()
{
  int numMotors = motorList.size();
  for (int i = 0; i < numMotors; i++)
  {
    mtsInterfaceRequired *required = this->AddInterfaceRequired(
      motorList[i] + "StateInterface");
    maxonInterface.push_back(new maxonMotorInterface());
    maxonInterface[i]->configureStateInterface(required);
  }
}

