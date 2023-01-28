// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>


class Flipper : public frc2::SubsystemBase {
 public:
  Flipper();

    frc2::InstantCommand UpCommand();
    frc2::InstantCommand DownCommand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Up();

  void Down();

 private:

  frc::DoubleSolenoid * m_pSolenoid;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
