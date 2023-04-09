// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include <ctre/Phoenix.h>


class Flipper : public frc2::SubsystemBase {
 public:
  Flipper();

    frc2::InstantCommand UpCommand(bool cone);
    frc2::InstantCommand AggitateCommand();
    frc2::InstantCommand DownCommand();
    frc2::InstantCommand LaunchCommand();
    frc2::InstantCommand FloatCommand();
    frc2::WaitUntilCommand WaitForFlipperFlipped();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Up(bool cone);

  void Aggitate();

  void Down();

  void LaunchCube();

  bool IsUp();
  
  bool IsHome();

  void Float();

  int GetActualPosition();

  void Zero();

 private:

  bool m_up;
  int m_desiredPosition;

  double m_topSoftLimit;

  TalonFX* m_pMotor;

  // frc::Solenoid * m_pSolenoid;
  // frc::Solenoid * m_pSolenoidFloat;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
