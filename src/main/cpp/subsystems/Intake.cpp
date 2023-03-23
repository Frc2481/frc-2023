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


frc2::InstantCommand Intake::TurnOffVerticalCommand(){
    return frc2::InstantCommand([this] {TurnOffVertical();},{this});
}

frc2::InstantCommand Intake::TurnOffHorizontalCommand(){
    return frc2::InstantCommand([this] {TurnOffHorizontal();},{this});
}

frc2::WaitUntilCommand Intake:: WaitForGamePieceCommand(){
    return frc2::WaitUntilCommand([this] {return HasGamePiece();});
}

Intake::Intake(){

    m_ExtendFirstSolenoid = new frc::DoubleSolenoid(
        frc::PneumaticsModuleType::CTREPCM, 
        SolenoidPorts::kIntakeFirstSolenoidPort,
        SolenoidPorts::kIntakeFirstSolenoidPortIn);
    
    // m_ExtendSecondSolenoid = new frc::Solenoid(
    //     frc::PneumaticsModuleType::CTREPCM, 
    //     SolenoidPorts::kIntakeSecondSolenoidPort);

    m_intakeBeambreak = new frc::DigitalInput(DigitalInputs::k_IntakeBeambreakPort);

    // m_compressor.EnableDigital();
    // m_compressor.EnableAnalog(units::pressure::pounds_per_square_inch_t(90),
    //                         units::pressure::pounds_per_square_inch_t(120));
    m_compressor.Enabled();
    
    // printf("Compressor Mode %d\n", m_compressor.GetConfigType());

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

    m_pHorizontalMotor->SetStatusFramePeriod(Status_1_General, 100, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_2_Feedback0, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_4_AinTempVbat, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_6_Misc, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_7_CommStatus, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_9_MotProfBuffer, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_10_MotionMagic, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_10_Targets, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_12_Feedback1, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_13_Base_PIDF0, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_14_Turn_PIDF1, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_15_FirmareApiStatus, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_17_Targets1, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_3_Quadrature, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_8_PulseWidth, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_11_UartGadgeteer, 255, 10);
    m_pHorizontalMotor->SetStatusFramePeriod(Status_Brushless_Current, 255, 10);

    // m_pHorizontalMotorFollower = new TalonFX(FalconIDs::kIntakeHorizontalMotorFollower);


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
    
    m_pVerticalMotor->SetStatusFramePeriod(Status_1_General, 100, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_2_Feedback0, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_4_AinTempVbat, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_6_Misc, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_7_CommStatus, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_9_MotProfBuffer, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_10_MotionMagic, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_10_Targets, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_12_Feedback1, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_13_Base_PIDF0, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_14_Turn_PIDF1, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_15_FirmareApiStatus, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_17_Targets1, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_3_Quadrature, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_8_PulseWidth, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_11_UartGadgeteer, 255, 10);
    m_pVerticalMotor->SetStatusFramePeriod(Status_Brushless_Current, 255, 10);
}

void Intake::TurnOnIntake(double horiz, double vert){
    m_pHorizontalMotor->Set(TalonFXControlMode::PercentOutput, 
                            horiz);
    m_pVerticalMotor->Set(TalonFXControlMode::PercentOutput, 
                          vert);
}

void Intake::TurnOnBarf(){
     m_pHorizontalMotor->Set(TalonFXControlMode::PercentOutput, 
                            IntakeConstants::k_IntakeBarfHorizontalRollerSpeed);
    m_pVerticalMotor->Set(TalonFXControlMode::PercentOutput, 
                          IntakeConstants::k_IntakeBarfVerticalRollerSpeed);
}

void Intake::TurnOff(){
    TurnOffHorizontal();
    TurnOffVertical();
}

void Intake::TurnOffVertical() {
    m_pVerticalMotor->Set(TalonFXControlMode::PercentOutput, 0);
}

void Intake::TurnOffHorizontal() {
    m_pHorizontalMotor->Set(TalonFXControlMode::PercentOutput, 0);
}

void Intake::Extend(){
    m_ExtendFirstSolenoid->Set(frc::DoubleSolenoid::kForward);
    // m_ExtendSecondSolenoid->Set(true);
    m_isExtended = true;
}

void Intake::Retract(){
    m_ExtendFirstSolenoid->Set(frc::DoubleSolenoid::kReverse);
    // m_ExtendSecondSolenoid->Set(false);
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
    // frc::SmartDashboard::PutNumber("Pressure", m_compressor.GetPressure().value());

    frc::SmartDashboard::PutNumber("Pressure", (250 * (m_analogPressure.GetAverageVoltage() / 5.0)) - 25);

    // static int loop_count = 0;
    // if (loop_count++ == 50) {
    //     m_compressor.EnableAnalog(units::pressure::pounds_per_square_inch_t(90),
    //                         units::pressure::pounds_per_square_inch_t(120));
    //     loop_count = 0;
    // }
}   
