// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Flipper.h"
#include "RobotParameters.h"


frc2::CommandPtr Flipper::UpCommand(){
    return RunOnce([this] {Up();});
}

frc2::CommandPtr Flipper::DownCommand(){
    return RunOnce([this] {Down();});
}

Flipper::Flipper(){
    m_pSolenoid = new frc::DoubleSolenoid(
        0, 
        frc::PneumaticsModuleType::CTREPCM, 
        SolenoidPorts::kFlipperSolenoidPort, 
        SolenoidPorts::kFlipperSolenoidReversePort);
}

void Flipper::Up(){
    m_pSolenoid->Set(m_pSolenoid->kForward);
 }

 void Flipper::Down(){
    m_pSolenoid->Set(m_pSolenoid->kReverse);
 }

// This method will be called once per scheduler run
void Flipper::Periodic() {}
