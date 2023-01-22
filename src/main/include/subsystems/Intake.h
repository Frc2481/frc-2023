// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/CommandPtr.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  frc2::CommandPtr ExtendCommand();
  frc2::CommandPtr RetractCommand();
  frc2::CommandPtr TurnOnIntakeCommand();
  frc2::CommandPtr TurnOnBarfCommand();
  frc2::CommandPtr TurnOffCommand();
  // frc2::CommandPtr WaitForGamePieceCommand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void TurnOnIntake();

  void TurnOnBarf();

  void TurnOff();

  void Extend();

  void Retract();

  bool IsExtended();

 private:
  TalonFX * m_pHorizontalMotor;
  TalonFX * m_pVerticalMotor;
  frc::DoubleSolenoid * m_ExtendSolenoid;
  bool m_isExtended;

 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.


};
