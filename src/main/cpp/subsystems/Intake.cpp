// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>

frc2::InstantCommand Intake::ExtendCommand(){
    return frc2::InstantCommand([this] {Extend();},{this});
}

frc2::InstantCommand Intake::RetractCommand(){
    return frc2::InstantCommand([this] {Retract();},{this});
}

frc2::InstantCommand Intake::TurnOnIntakeCommand(){
    return frc2::InstantCommand([this] {TurnOnIntake();},{this});
}

frc2::InstantCommand Intake::TurnOnBarfCommand(){
    return frc2::InstantCommand([this] {TurnOnBarf();},{this});
}

frc2::InstantCommand Intake::TurnOffCommand(){
    return frc2::InstantCommand([this] {TurnOff();},{this});
}

frc2::WaitUntilCommand Intake::WaitForGamePieceCommand(){
    return frc2::WaitUntilCommand([this] {return HasGamePiece();});
}

Intake::Intake(){
    m_ExtendFirstSolenoid = new frc::Solenoid(
        frc::PneumaticsModuleType::REVPH, 
        SolenoidPorts::kIntakeFirstSolenoidPort);
    
    m_ExtendSecondSolenoid = new frc::Solenoid(
        frc::PneumaticsModuleType::REVPH, 
        SolenoidPorts::kIntakeSecondSolenoidPort);

    m_intakeBeambreak = new frc::DigitalInput(DigitalInputs::k_IntakeBeambreakPort);

    m_pHorizontalMotor = new TalonFX(FalconIDs::kIntakeHorizontalMotor);
    m_pHorizontalMotor->ConfigFactoryDefault();
    m_pHorizontalMotor->SetNeutralMode(NeutralMode::Brake);
    m_pHorizontalMotor->EnableVoltageCompensation(true);
    m_pHorizontalMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_pHorizontalMotor->ConfigNeutralDeadband(0.04, 0);
    m_pHorizontalMotor->ConfigNominalOutputForward(0.0, 0.0);
    m_pHorizontalMotor->ConfigNominalOutputReverse(0.0, 0.0);
    m_pHorizontalMotor->ConfigPeakOutputForward(1.0, 0.0);
    m_pHorizontalMotor->ConfigPeakOutputReverse(-1.0, 0.0);
    m_pHorizontalMotor->SetSensorPhase(false);	
    m_pHorizontalMotor->SetInverted(false);
    m_pHorizontalMotor->ConfigSupplyCurrentLimit(
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(
            true, 
            IntakeConstants::k_IntakeHorizontalCurrentLimit - 5, 
            IntakeConstants::k_IntakeHorizontalCurrentLimit, 
            IntakeConstants::k_IntakeCurrentDuration), 0);

    m_pVerticalMotor = new TalonFX(FalconIDs::kIntakeVerticalMotor);
    m_pVerticalMotor->ConfigFactoryDefault();
    m_pVerticalMotor->SetNeutralMode(NeutralMode::Brake);
    m_pVerticalMotor->EnableVoltageCompensation(true);
    m_pVerticalMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_pVerticalMotor->ConfigNeutralDeadband(0.04, 0);
    m_pVerticalMotor->ConfigNominalOutputForward(0.0, 0.0);
    m_pVerticalMotor->ConfigNominalOutputReverse(0.0, 0.0);
    m_pVerticalMotor->ConfigPeakOutputForward(1.0, 0.0);
    m_pVerticalMotor->ConfigPeakOutputReverse(-1.0, 0.0);
    m_pVerticalMotor->SetSensorPhase(false);	
    m_pVerticalMotor->SetInverted(false);
    m_pVerticalMotor->ConfigSupplyCurrentLimit(
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(
            true, 
            IntakeConstants::k_IntakeVerticalCurrentLimit - 5, 
            IntakeConstants::k_IntakeVerticalCurrentLimit, 
            IntakeConstants::k_IntakeCurrentDuration), 0);
    
}

void Intake::TurnOnIntake(){
    m_pHorizontalMotor->Set(TalonFXControlMode::PercentOutput, 
                            IntakeConstants::k_IntakeHorizontalRollerSpeed);
    m_pVerticalMotor->Set(TalonFXControlMode::PercentOutput, 
                          IntakeConstants::k_IntakeVerticalRollerSpeed);
}

void Intake::TurnOnBarf(){
     m_pHorizontalMotor->Set(TalonFXControlMode::PercentOutput, 
                            IntakeConstants::k_IntakeBarfHorizontalRollerSpeed);
    m_pVerticalMotor->Set(TalonFXControlMode::PercentOutput, 
                          IntakeConstants::k_IntakeBarfVerticalRollerSpeed);
}

void Intake::TurnOff(){
    m_pHorizontalMotor->Set(TalonFXControlMode::PercentOutput, 0);
    m_pVerticalMotor->Set(TalonFXControlMode::PercentOutput, 0);
}

void Intake::Extend(){
    m_ExtendFirstSolenoid->Set(true);
    m_ExtendSecondSolenoid->Set(true);
    m_isExtended = true;
}

void Intake::Retract(){
    m_ExtendFirstSolenoid->Set(false);
    m_ExtendSecondSolenoid->Set(false);
    m_isExtended = false;
}

bool Intake::IsExtended(){
    return m_isExtended;
}

bool Intake::HasGamePiece(){
  return !m_intakeBeambreak->Get();
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Intake Beam Break", HasGamePiece());
}
