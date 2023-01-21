#include "subsystems/Slide.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include "networktables/NetworkTableInstance.h"

frc2::CommandPtr Slide::GoToCenterPositionCommand(){
    return GoToPositionCommand(SlideConstants::k_SlideCenterPosition);
}

frc2::CommandPtr Slide::TrackLimelightTopPostCommand(){
    return frc2::FunctionalCommand(
        [this] {
            nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", LimeLightConstants::k_TopPostPipeLine);
        }, 
        [this]{
            double x_in = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0);
            double y_in = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0);
            double d_in = (LimeLightConstants::k_TopPostHeight_in - LimeLightConstants::k_CameraHeight_in) / tan(y_in * std::numbers::pi / 180.0);
            double SlideOffset_in = d_in * sin(x_in * std::numbers::pi / 180);
            SetTargetPosition(SlideOffset_in);
        }, 
        [this](bool interrupted){
            SetTargetPosition(0.0);
            nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", LimeLightConstants::k_MarkersPipeline);
        },
        [this]{
            return false;
        },{this}
    ).ToPtr();
}

frc2::CommandPtr Slide::TrackLimelightBottomPostCommand(){
    return frc2::FunctionalCommand(
        [this] {
            nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", LimeLightConstants::k_BottomPostPipeLine);
        }, 
        [this]{
            double x_in = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0);
            double y_in = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0);
            double d_in = (LimeLightConstants::k_BottomPostHeight_in - LimeLightConstants::k_CameraHeight_in) / tan(y_in * std::numbers::pi / 180.0);
            double SlideOffset_in = d_in * sin(x_in * std::numbers::pi / 180);
            SetTargetPosition(SlideOffset_in);
        }, 
        [this](bool interrupted){
            SetTargetPosition(0.0);
            nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", LimeLightConstants::k_MarkersPipeline);
        },
        [this]{
            return false;
        },{this}
    ).ToPtr();
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
    m_pMotor->ConfigMotionCruiseVelocity(SlideConstants::k_SlideMaxSpeed);  //Degrees per second
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
    m_pMotor->ConfigForwardSoftLimitThreshold(SlideConstants::k_SlideLeftSoftLimit, 10);
    m_pMotor->ConfigReverseSoftLimitThreshold(SlideConstants::k_SlideRightSoftLimit, 10);
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