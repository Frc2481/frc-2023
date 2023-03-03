// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gripper.h"
#include "RobotParameters.h"

 frc2::InstantCommand Gripper::OpenCommand(){
    return frc2::InstantCommand([this] {Open();});
 }

 frc2::InstantCommand Gripper::CloseCommand(){
    return frc2::InstantCommand([this] {Close();});
 }

 frc2::InstantCommand Gripper::PickedUpCubeCommand(){
   return frc2::InstantCommand([this] {m_GamePiece = CUBE;});
 }

 frc2::InstantCommand Gripper::PickedUpConeCommand(){
   return frc2::InstantCommand([this] {m_GamePiece = CONE;});
 }

 frc2::InstantCommand Gripper::DroppedGamePieceCommand(){
   return frc2::InstantCommand([this] {m_GamePiece = NONE;});
 }
 
Gripper::Gripper(){
   m_pSolenoid = new frc::DoubleSolenoid(
        frc::PneumaticsModuleType::REVPH, 
        SolenoidPorts::kGripperSolenoidPort, 
        SolenoidPorts::kGripperSolenoidReversePort);
}

 void Gripper::Open(){
    m_pSolenoid->Set(m_pSolenoid->kForward);
 }

 void Gripper::Close(){
    m_pSolenoid->Set(m_pSolenoid->kReverse);
 }

  GamePieceType Gripper::GetGamePieceType(){
    return m_GamePiece;
 }

// This method will be called once per scheduler run
void Gripper::Periodic() {}
