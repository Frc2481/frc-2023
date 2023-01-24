// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gripper.h"
#include "RobotParameters.h"

 frc2::CommandPtr Gripper::OpenCommand(){
    return RunOnce([this] {Open();});
 }

 frc2::CommandPtr Gripper::CloseCommand(){
    return RunOnce([this] {Close();});
 }

Gripper::Gripper(){
   m_pSolenoid = new frc::DoubleSolenoid(
        0, 
        frc::PneumaticsModuleType::CTREPCM, 
        SolenoidPorts::kGripperSolenoidPort, 
        SolenoidPorts::kGripperSolenoidReversePort);
}

 void Gripper::Open(){
    m_pSolenoid->Set(m_pSolenoid->kForward);
 }

 void Gripper::Close(){
    m_pSolenoid->Set(m_pSolenoid->kReverse);
 }

// This method will be called once per scheduler run
void Gripper::Periodic() {}