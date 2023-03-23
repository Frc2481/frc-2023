// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include "RobotParameters.h"
#include <frc/Compressor.h>
#include <frc/AnalogInput.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  frc2::InstantCommand ExtendCommand();
  frc2::InstantCommand RetractCommand();
  frc2::InstantCommand TurnOnIntakeCommand();
  frc2::InstantCommand TurnOnBarfCommand();
  frc2::InstantCommand TurnOffCommand();
  frc2::InstantCommand TurnOffHorizontalCommand();
  frc2::InstantCommand TurnOffVerticalCommand();
  
  frc2::WaitUntilCommand WaitForGamePieceCommand();

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void TurnOnIntake(double horiz = IntakeConstants::k_IntakeHorizontalRollerSpeed, double vert = IntakeConstants::k_IntakeVerticalRollerSpeed);

  void TurnOnBarf();

  void TurnOff();
  void TurnOffVertical();
  void TurnOffHorizontal();

  void Extend();

  void Retract();

  bool IsExtended();

  bool HasGamePiece();

 private:
  TalonFX * m_pHorizontalMotor;
  TalonFX * m_pHorizontalMotorFollower;
  TalonFX * m_pVerticalMotor;
  frc::DoubleSolenoid * m_ExtendFirstSolenoid;
  frc::Solenoid * m_ExtendSecondSolenoid;
  bool m_isExtended;
  frc::DigitalInput * m_intakeBeambreak;
  frc::Compressor m_compressor{frc::PneumaticsModuleType::CTREPCM};
  frc::AnalogInput m_analogPressure{3};

 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.


};
