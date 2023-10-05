#ifndef _MAXON_STATUS_WIDGET_H
#define _MAXON_STATUS_WIDGET_H

#include <QWidget>
#include <maxonControl/maxonMotor.h>
#include <maxonControl/maxonMotorInterface.h>

class QLineEdit;

/*! 
 * This class generates a status widget to display maxon information.
 *
 * Example use would be to create a timer querying the status, then call the updateStatus() method.
 */
class maxonStatusWidget : public QWidget, public mtsComponent
{
  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)

  // attributes
protected:
  QLineEdit *enabled, *opMode;
  QLineEdit *position, *velocity, *velocityAveraged;
#ifdef MAXON_DISPLAY_RAW
  QLineEdit *positionRaw, *velocityRaw, *velocityAveragedRaw;
#endif
  QLineEdit *current, *currentAveraged;
  QLineEdit *force;

  std::vector<maxonMotorInterface*> maxonInterface;
  std::vector<std::string> motorList;

  // methods
public:
  maxonStatusWidget(const std::vector<std::string> &list, const std::string &taskName="maxonStatusWidget", QWidget *parent = 0);
  ~maxonStatusWidget();

  bool update(int i);

  void Connect(mtsManagerLocal *localManager);

protected:
  void setupUi();
  void setupMultiTask();

  int numMotors;
};

CMN_DECLARE_SERVICES_INSTANTIATION(maxonStatusWidget)

#endif // _MAXON_STATUS_WIDGET_H
