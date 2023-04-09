// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Flipper.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>


frc2::InstantCommand Flipper::UpCommand(bool cone){
    return frc2::InstantCommand([this, cone] {Up(cone);});
}

frc2::InstantCommand Flipper::AggitateCommand() {
    return frc2::InstantCommand([this] {Aggitate();});
}

frc2::InstantCommand Flipper::DownCommand(){
    return frc2::InstantCommand([this] {Down();});
}   

frc2::InstantCommand Flipper::LaunchCommand(){
    return frc2::InstantCommand([this] {LaunchCube();});
}  

frc2::InstantCommand Flipper::FloatCommand() {
    return frc2::InstantCommand([this] {Float();});
}

frc2::WaitUntilCommand Flipper::WaitForFlipperFlipped() {
    return frc2::WaitUntilCommand([this] { return IsUp(); });
}

Flipper::Flipper(){
    m_pMotor = new TalonFX(FalconIDs::kFlipperMotorID);

    m_pMotor->ConfigFactoryDefault();
    m_pMotor->ConfigClearPositionOnLimitR(true);
    m_pMotor->SetNeutralMode(NeutralMode::Brake);
    m_pMotor->EnableVoltageCompensation(true);
    m_pMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_pMotor->ConfigNeutralDeadband(0.04, 0);
    m_pMotor->ConfigNominalOutputForward(0.0, 0.0);
    m_pMotor->ConfigNominalOutputReverse(-0.3, 0.0);
    m_pMotor->ConfigPeakOutputForward(1.0, 0.0);
    m_pMotor->ConfigPeakOutputReverse(-1.0, 0.0);
    // m_pMotor->SetSensorPhase(true);	
    m_pMotor->SetInverted(true);
    m_pMotor->ConfigForwardSoftLimitThreshold(FlipperConstants::k_FlipperTopSoftLimit, 10);
    m_pMotor->ConfigReverseSoftLimitThreshold(FlipperConstants::k_FlipperBottomSoftLimit, 10);
    m_pMotor->ConfigForwardSoftLimitEnable(true, 10);
    m_pMotor->ConfigReverseSoftLimitEnable(true, 10);
    m_pMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    
    SupplyCurrentLimitConfiguration supplyCurrentConfig;
    supplyCurrentConfig.currentLimit = 5.0;
    supplyCurrentConfig.triggerThresholdTime = 0.25;
    supplyCurrentConfig.triggerThresholdCurrent = 25.0;
    m_pMotor->ConfigSupplyCurrentLimit(supplyCurrentConfig, 10);
    
}

void Flipper::Up(bool cube){
    m_topSoftLimit = cube ? FlipperConstants::k_FlipperCubeTopSoftLimit : FlipperConstants::k_FlipperTopSoftLimit;
    m_pMotor->ConfigForwardSoftLimitThreshold(m_topSoftLimit, 10);
    m_pMotor->Set(ControlMode::PercentOutput, cube ? FlipperConstants::k_FlipperCubeSpeed : FlipperConstants::k_FlipperConeSpeed);
 }

void Flipper::Aggitate() {
    m_topSoftLimit = FlipperConstants::k_FlipperAggitateTopSoftLimit;
    m_pMotor->ConfigForwardSoftLimitThreshold(m_topSoftLimit, 10);
    m_pMotor->Set(ControlMode::PercentOutput, FlipperConstants::k_FlipperCubeSpeed);
}

void Flipper::Down(){
    m_pMotor->Set(ControlMode::PercentOutput, FlipperConstants::k_FlipperDownSpeed);
 }

void Flipper::LaunchCube(){
    m_pMotor->Set(ControlMode::PercentOutput, FlipperConstants::k_FlipperCubeLaunchSpeed);
}

int Flipper::GetActualPosition() {
    return m_pMotor->GetSelectedSensorPosition();
}

void Flipper::Zero(){
    m_pMotor->SetSelectedSensorPosition(0, 0, 10);
}

 bool Flipper::IsUp() {
    // Check for at least 90% of the flipped position to be considered up. 
    return GetActualPosition() > m_topSoftLimit * 0.90;
 }

 bool Flipper::IsHome() {
    // Check for at least 110% of the home position to be considered up. 
    return GetActualPosition() < FlipperConstants::k_FlipperTopSoftLimit * 0.10;
 }

 void Flipper::Float() {
    // It's a Falcon FX... Falcon's can't float.  Very small rocks float...
 }

 
// This method will be called once per scheduler run
void Flipper::Periodic() {
    frc::SmartDashboard::PutNumber("Flipper Actual Position", GetActualPosition());
    frc::SmartDashboard::PutNumber("Flipper Stator Current", m_pMotor->GetStatorCurrent());
    frc::SmartDashboard::PutNumber("Flipper Supply Current", m_pMotor->GetSupplyCurrent());
}
