// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "components/CommonMotorController.h"

#pragma once

class SparkMaxMotorController : public CommonMotorController{
 private:
  rev::CANSparkMax* m_pMotor;
  double m_setpoint = 0.0;
  rev::CANSparkMax::ControlType m_pCurrentMode;
 public:
  SparkMaxMotorController(int motorID, const std::string &name, rev::CANSparkMax::MotorType type);
  void  Config_kF(int slotIdx, double value, int timeoutMs = 0);
	void  Config_kP(int slotIdx, double value, int timeoutMs = 0);
	void  Config_kI(int slotIdx, double value, int timeoutMs = 0);
  void  Config_kD(int slotIdx, double value, int timeoutMs = 0);
  void  Config_IntegralZone(int slotIdx, int izone, int timeoutMs = 0);
  void  ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs = 0);
  void  EnableVoltageCompensation(bool enable);
  void  SetInverted(bool isInverted);
  void  Set(double speed);
  void  Set(CommonModes mode, double value);
  void  Set(CommonModes mode, double demand0, DemandType demand1Type, double demand1);
  // void Set(ctre::phoenix::motorcontrol::ControlMode mode, double demand0, double demand1);
  void ConfigFactoryDefault();
  double GetVelocity();
  void SetEncoderPosition(double pos);
  void SetVelocityConversionFactor(double factor);
  double GetClosedLoopError();
  bool CommonModesToControlType(CommonModes mode, rev::CANSparkMax::ControlType &retMode);
  void SetNeutralMode(rev::CANSparkMax::IdleMode mode);
  double GetPos();
  
};
