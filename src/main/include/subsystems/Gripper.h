// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

enum GamePieceType{CUBE, CONE, NONE};
class Gripper : public frc2::SubsystemBase {
 public:
  Gripper();

   frc2::InstantCommand OpenCommand();
   frc2::InstantCommand CloseCommand();
   frc2::InstantCommand PickedUpCubeCommand();
   frc2::InstantCommand PickedUpConeCommand();
   frc2::InstantCommand DroppedGamePieceCommand();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Open();

  void Close();

  GamePieceType GetGamePieceType();

 private:

 frc::DoubleSolenoid * m_pSolenoid;
 GamePieceType m_GamePiece;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
