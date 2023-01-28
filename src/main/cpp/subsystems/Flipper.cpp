// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Flipper.h"
#include "RobotParameters.h"


frc2::InstantCommand Flipper::UpCommand(){
    return frc2::InstantCommand([this] {Up();});
}

frc2::InstantCommand Flipper::DownCommand(){
    return frc2::InstantCommand([this] {Down();});
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
