#ifndef _MAXON_CONTROL_WIDGET_H
#define _MAXON_CONTROL_WIDGET_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QCheckBox>

#include <cisstMultiTask/mtsComponent.h>


/*!
 * This class creates a control widget for a maxon motor
 */

class maxonControlWidget : public QWidget, public mtsComponent
{
  Q_OBJECT
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT)

  // attributes
protected:
  QDoubleSpinBox *vel;
  QDoubleSpinBox *homingForce;
  QDoubleSpinBox *velocityPointBox;
  //QDoubleSpinBox *acc, *dec;
  QGroupBox *velGroup, *posGroup;
  QCheckBox *absoluteMove;
  QDoubleSpinBox *position;

  /// If true, use inputs
  bool useRaw;

  /// cisst struct for maxon motor
  struct {
    mtsFunctionVoidReturn enable;
    mtsFunctionVoid disable;
    mtsFunctionVoid clearFault;
    mtsFunctionVoid stop;
    mtsFunctionVoidReturn getTargetVelocity;
    mtsFunctionVoidReturn getTargetVelocity_raw;
    mtsFunctionWrite setVelocity;
    mtsFunctionWrite setVelocity_raw;
    mtsFunctionWrite setVelocityPoint;
    mtsFunctionWrite setVelocityPoint_raw;
    mtsFunctionVoid jogPlus;
    mtsFunctionVoid jogMinus;
    mtsFunctionWrite moveToAbsolutePosition;
    mtsFunctionWrite moveToAbsolutePosition_raw;
    mtsFunctionWrite moveToRelativePosition;
    mtsFunctionWrite moveToRelativePosition_raw;
    mtsFunctionWrite setSetPointMode;
    mtsFunctionVoid setPositionMode;
    mtsFunctionVoid setVelocityMode;
    mtsFunctionVoidReturn getErrorString;
    mtsFunctionRead getNodeId;
    mtsFunctionVoid zero;
    mtsFunctionVoid moveToZero;
    mtsFunctionRead negativeLimit;
    mtsFunctionRead positiveLimit;
    mtsFunctionWrite setNegativeLimit;
    mtsFunctionWrite setDigitalOutput;
    //mtsFunctionWrite setPositiveLimit;
    mtsFunctionVoid setNegativeLimitCurrentPosition;
    //mtsFunctionVoid setPositiveLimitCurrentPosition;
    mtsFunctionWrite setHomeForce;
    mtsFunctionVoid startForceHoming;
    mtsFunctionRead getCableForce;
    mtsDouble cableForce;
    mtsFunctionVoid rebias;
  } maxon;

  // methods
public:
  /// Constructor
  /// \param motorName The MTS name for the motor. This creates a new mtsComponent
  ///   with the name motorName + "Widget".
  /// \param parent The Qt parent
  maxonControlWidget(const std::string &motorName, QWidget *parent = 0);
  ~maxonControlWidget();

public slots:
  void on_enable_toggled(bool checked);
  void on_rebias_clicked();
  void on_homeForce_valueChanged(double val); 
  void on_home_clicked();
  void on_clearFault_clicked();
  void on_opModeTabWidget_currentChanged(int id);
  void on_velocity_valueChanged(double val);
  void on_velocityPointBox_valueChanged(double val);
  void on_jogPlus_clicked();
  void on_jogMinus_clicked();
  void on_movePosition_clicked();
  void on_stop_clicked();
  void on_displayUnits_clicked(bool checked);
  void on_zero_clicked();
  void on_moveToZero_clicked();
  void on_negLimit_clicked();
  //void on_posLimit_clicked();

protected:
  void setupUi();

};

CMN_DECLARE_SERVICES_INSTANTIATION(maxonControlWidget)

#endif // _MAXON_CONTROL_WIDGET_H
