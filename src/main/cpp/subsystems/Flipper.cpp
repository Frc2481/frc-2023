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
    m_pSolenoid = new frc::Solenoid( 
        frc::PneumaticsModuleType::REVPH, 
        SolenoidPorts::kFlipperSolenoidPort);
}

void Flipper::Up(){
    m_up = true;
    m_pSolenoid->Set(true);
 }

 void Flipper::Down(){
    m_up = false;
    m_pSolenoid->Set(false);
 }

 bool Flipper::IsUp() {
    return m_up;
 }

// This method will be called once per scheduler run
void Flipper::Periodic() {}
