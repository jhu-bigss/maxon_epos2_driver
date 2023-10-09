#ifndef _MAXON_WIDGET_H
#define _MAXON_WIDGET_H

/*!
 * This class defines a widget for basic control of a maxon motor
 */

#include <QWidget>
#include <vector>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassServices.h>
#include <cisstCommon/cmnClassRegisterMacros.h>

class maxonInterface;
class maxonControlWidget;
class maxonStatusWidget;
class QTimer;

class mtsManagerLocal;

class maxonWidget : public QWidget, public cmnGenericObject
{
  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)

  // attributes
protected:
  maxonInterface *maxon;
  QTimer *timer;

  int motorIndex;

  // methods
public:
  maxonWidget(maxonInterface *maxon_, QWidget *parent = 0);
  virtual ~maxonWidget();

  maxonStatusWidget *statusWidget;
  std::vector< maxonControlWidget *> controls;

  void Connect(mtsManagerLocal *localManager);


public slots:
  void on_selectNode_currentIndexChanged(int index);
  void on_recordData_toggled(bool checked);
  void on_timer_timeout();

protected:
  void setupUi();
};

CMN_DECLARE_SERVICES_INSTANTIATION(maxonWidget)

#endif // _MAXON_WIDGET_H
