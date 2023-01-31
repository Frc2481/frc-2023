// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "RobotParameters.h"

#pragma once

class CommonMotorController {
 private:
  std::string m_name;
 public:
  CommonMotorController(int motorID, const std::string name){
    m_name = name;
  }
  virtual void  SetStatusFramePeriod(StatusFrameEnhanced frame, uint8_t periodMs, int timeoutMs = 0){
    printf("Warning: %s is calling SetStatusFramePeriod() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigMotionAcceleration(int sensorUnitsPer100ms, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigMotionAcceleration() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigMotionSCurveStrength(int curveStrength, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigMotionSCurveStrength() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigMotionCruiseVelocity() which is not implemented\n", m_name.c_str());
  }
  virtual void  Config_kF(int slotIdx, double value, int timeoutMs = 0){
    printf("Warning: %s is calling Config_kF() which is not implemented\n", m_name.c_str());
  }
  virtual void  config_PID(){
    printf("Warning: %s is calling config_PID() which is not implemented\n", m_name.c_str());
  }//TODO fix
  virtual void  SelectProfileSlot(int slotIdx, int pidIdx){
    printf("Warning: %s is calling SelectProfileSlot() which is not implemented\n", m_name.c_str());
  }
	virtual void  Config_kP(int slotIdx, double value, int timeoutMs = 0){
    printf("Warning: %s is calling Config_kP() which is not implemented\n", m_name.c_str());
  }
	virtual void  Config_kI(int slotIdx, double value, int timeoutMs = 0){
    printf("Warning: %s is calling Config_kI() which is not implemented\n", m_name.c_str());
  }
	virtual void  Config_kD(int slotIdx, double value, int timeoutMs = 0){
    printf("Warning: %s is calling Config_kD() which is not implemented\n", m_name.c_str());
  }
  virtual void  Config_IntegralZone(int slotIdx, int izone, int timeoutMs = 0){
    printf("Warning: %s is calling Config_IntegralZone() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigMaxIntegralAccumulator() which is not implemented\n", m_name.c_str());
  }
  virtual void  SetNeutralMode(NeutralMode neutralMode){
    printf("Warning: %s is calling SetNeutralMode() which is not implemented\n", m_name.c_str());
  }
  virtual void SetNeutralMode(rev::CANSparkMax::IdleMode mode){
    printf("Warning: %s is calling SetNeutralMode() which is not implemented\n", m_name.c_str());
  }
  virtual void  EnableVoltageCompensation(bool enable){
    printf("Warning: %s is calling EnableVoltageCompensation() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigVoltageCompSaturation(double voltage, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigVoltageCompSaturation() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigNeutralDeadband(double percentDeadband, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigNeutralDeadband() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigNominalOutputForward(double percentOut, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigNominalOutputForward() which is not implemented\n", m_name.c_str());
  }
	virtual void  ConfigNominalOutputReverse(double percentOut, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigNominalOutputReverse() which is not implemented\n", m_name.c_str());
  }
	virtual void  ConfigPeakOutputForward(double percentOut, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigPeakOutputForward() which is not implemented\n", m_name.c_str());
  }
	virtual void  ConfigPeakOutputReverse(double percentOut, int timeoutMs = 0){
    printf("Warning: %s is calling ConfigPeakOutputReverse() which is not implemented\n", m_name.c_str());
  }
	virtual void  SetSensorPhase(bool PhaseSensor){
    printf("Warning: %s is calling SetSensorPhase() which is not implemented\n", m_name.c_str());
  }
	virtual void  SetInverted(bool isInverted){
    printf("Warning: %s is calling SetInverted() which is not implemented\n", m_name.c_str());
  }
  virtual void  Set(double speed){
    printf("Warning: %s is calling Set() which is not implemented\n", m_name.c_str());
  }
  virtual void  Set(CommonModes mode, double value){
    printf("Warning: %s is calling Set() which is not implemented\n", m_name.c_str());
  }
  virtual void  Set(CommonModes mode, double demand0, DemandType demand1Type, double demand1){
    printf("Warning: %s is calling Set() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigFactoryDefault(){
    printf("Warning: %s is calling ConfigFactoryDefault() which is not implemented\n", m_name.c_str());
  }
  virtual void  ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs){
    printf("Warning: %s is calling ConfigSelectedFeedbackSensor() which is not implemented\n", m_name.c_str());
  }
  virtual int GetSelectedSensorPosition(int id){
    printf("Warning: %s is calling GetSelectedSensorPosition() which is not implemented\n", m_name.c_str());
    return 0;
  }
  virtual int isSensorConnected(){
    printf("Warning: %s is calling isSensorConnected() which is not implemented\n", m_name.c_str());
    return 1;
  }
  virtual double GetVelocity(){
    printf("Warning: %s is calling GetVelocity() which is not implemented\n", m_name.c_str());
    return 0.0;
  }
  virtual void SetEncoderPosition(double pos){
    printf("Warning: %s is calling SetEncoderPosition() which is not implemented\n", m_name.c_str());
  }
  virtual void SetVelocityConversionFactor(double factor){
    printf("Warning: %s is calling SetEncoderPosition() which is not implemented\n", m_name.c_str());
  }
  //add to other motors
  virtual double GetClosedLoopError(){
    printf("Warning: %s is calling GetClosedLoopError() which is not implemented\n", m_name.c_str());
    return 0.0;
  }
  virtual double GetPos(){
    printf("Warning: %s is calling GetPos() which is not implemented\n", m_name.c_str());
    return 0.0;
  }
};