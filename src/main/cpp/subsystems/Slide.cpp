#include "subsystems/Slide.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>

frc2::CommandPtr Slide::GoToCenterPositionCommand(){
    return GoToPositionCommand(SlideConstants::k_SlideCenterPosition);
}

frc2::CommandPtr Slide::TrackLimelightUpperPostCommand(){
    return ;
}

frc2::CommandPtr Slide::TrackLimelightLowerPostCommand(){
    return ;
}

frc2::CommandPtr Slide::GoToPositionCommand(double pos){
    return RunOnce([this, pos] {SetTargetPosition(pos);});
}

frc2::CommandPtr Slide::WaitForSlideOnTargetCommand(){
    return frc2::WaitUntilCommand([this] {return IsOnTarget();}).ToPtr();
}

Slide::Slide(){
    m_pMotor = new TalonFX(FalconIDs::kSlideMotor);
    m_pMotor->ConfigFactoryDefault();
    m_pMotor->Config_kP(0, SlideConstants::k_SlidekP, 10);
    m_pMotor->Config_kI(0, SlideConstants::k_SlidekI, 10);
    m_pMotor->Config_kD(0, SlideConstants::k_SlidekD, 10);
    m_pMotor->Config_kF(0, SlideConstants::k_SlidekF, 10);
    m_pMotor->Config_IntegralZone(0, 0, 10);
    m_pMotor->ConfigMaxIntegralAccumulator (0, 0, 10);
    m_pMotor->ConfigMotionCruiseVelocity(SlideConstants::k_MaxSlideSpeed);  //Degrees per second
    m_pMotor->ConfigMotionAcceleration(SlideConstants::k_SlideAcceleration);
    m_pMotor->ConfigMotionSCurveStrength(SlideConstants::k_SlideSCurveStrength);
    m_pMotor->SetNeutralMode(NeutralMode::Brake);
    m_pMotor->EnableVoltageCompensation(true);
    m_pMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_pMotor->ConfigNeutralDeadband(0.04, 0);
    m_pMotor->ConfigNominalOutputForward(0.0, 0.0);
    m_pMotor->ConfigNominalOutputReverse(0.0, 0.0);
    m_pMotor->ConfigPeakOutputForward(1.0, 0.0);
    m_pMotor->ConfigPeakOutputReverse(-1.0, 0.0);
    m_pMotor->SetSensorPhase(false);	
    m_pMotor->SetInverted(false);
    m_pMotor->ConfigForwardSoftLimitThreshold(SlideConstants::k_SlideTopSoftLimit, 10);
    m_pMotor->ConfigReverseSoftLimitThreshold(SlideConstants::k_SlideBottomSoftLimit, 10);
    m_pMotor->ConfigForwardSoftLimitEnable(true, 10);
    m_pMotor->ConfigReverseSoftLimitEnable(true, 10);
}

void Slide::Periodic(){
    frc::SmartDashboard::PutNumber("Slide Target Position", GetTargetPosition());
    frc::SmartDashboard::PutNumber("Slide Actual Position", GetActualPosition());
    frc::SmartDashboard::PutBoolean("Slide On Target", IsOnTarget());
}

void Slide::SetTargetPosition(double pos){
    m_desiredPosition = pos;
    double ticks = pos * SlideConstants::k_SlideTicksPerInch;
    m_pMotor->Set(TalonFXControlMode::MotionMagic, ticks);

}

double Slide::GetTargetPosition(){
    return m_desiredPosition;
}

double Slide::GetActualPosition(){
    return m_pMotor->GetSelectedSensorPosition() / SlideConstants::k_SlideTicksPerInch;
}

void Slide::Zero(){
    m_pMotor->SetSelectedSensorPosition(0, 0, 10);
}

bool Slide::IsOnTarget(){
    return abs(GetActualPosition() - GetTargetPosition()) < SlideConstants::k_SlideOnTargetThreshold;
}