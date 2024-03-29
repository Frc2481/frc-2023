#include "subsystems/Elevator.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/Preferences.h>

frc2::InstantCommand Elevator::StowCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorStowPosition);
}

frc2::InstantCommand Elevator::GoToFloorCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorFloorPosition);
}

frc2::InstantCommand Elevator::GoToBottomPostCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorMidPosition);
}

frc2::InstantCommand Elevator::GoToTopPostCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorTopPosition);
}

frc2::InstantCommand Elevator::GoToPositionCommand(double pos){
    return frc2::InstantCommand([this, pos] {SetTargetPosition(pos);},{this});
}

frc2::WaitUntilCommand Elevator::WaitForElevatorOnTargetCommand(){
    return frc2::WaitUntilCommand([this] {return IsOnTarget();});
}

frc2::WaitUntilCommand Elevator::WaitForElevatorPastPositionCommand(){
    return frc2::WaitUntilCommand([this] {return GetActualPosition() > (ElevatorConstants::k_ElevatorTopPosition * 0.90);});
}

frc2::WaitUntilCommand Elevator::WaitForElevatorPastMidPositionCommand(){
    return frc2::WaitUntilCommand([this] {return GetActualPosition() > (ElevatorConstants::k_ElevatorMidPosition * 0.90);});
}

frc2::InstantCommand Elevator::EngageBrakeCommand(){
    return frc2::InstantCommand([this] {EngageBrake();},{this});
}

frc2::InstantCommand Elevator::ReleaseBrakeCommand(){
    return frc2::InstantCommand([this] {ReleaseBrake();},{this});
}

Elevator::Elevator(){

     m_brakeSolenoid = new frc::DoubleSolenoid(
        frc::PneumaticsModuleType::CTREPCM, 
        SolenoidPorts::kElevatorEngageBrakePort, 
        SolenoidPorts::kElevatorReleaseBrakePort);

    m_elevatorBeambreak = new frc::DigitalInput(DigitalInputs::k_ElevatorBeambreakPort);

    m_pMotor = new TalonFX(FalconIDs::kElevatorMotor);
    m_pMotor->ConfigFactoryDefault();
    m_pMotor->SelectProfileSlot(0, 0);
    m_pMotor->Config_kP(0, ElevatorConstants::k_ElevatorkP, 10);
    m_pMotor->Config_kI(0, ElevatorConstants::k_ElevatorkI, 10);
    m_pMotor->Config_kD(0, ElevatorConstants::k_ElevatorkD, 10);
    m_pMotor->Config_kF(0, ElevatorConstants::k_ElevatorkF, 10);
    m_pMotor->Config_IntegralZone(0, 0, 10);
    m_pMotor->ConfigMaxIntegralAccumulator (0, 0, 10);
    m_pMotor->ConfigMotionCruiseVelocity(ElevatorConstants::k_ElevatorMaxSpeed);  //Degrees per second
    m_pMotor->ConfigMotionAcceleration(ElevatorConstants::k_ElevatorAcceleration);
    m_pMotor->ConfigMotionSCurveStrength(ElevatorConstants::k_ElevatorSCurveStrength);
    m_pMotor->SetNeutralMode(NeutralMode::Brake);
    m_pMotor->EnableVoltageCompensation(true);
    m_pMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_pMotor->ConfigNeutralDeadband(0.04, 0);
    m_pMotor->ConfigNominalOutputForward(0.0, 0.0);
    m_pMotor->ConfigNominalOutputReverse(-0.3, 0.0);
    m_pMotor->ConfigPeakOutputForward(1.0, 0.0);
    m_pMotor->ConfigPeakOutputReverse(-1.0, 0.0);
    // m_pMotor->SetSensorPhase(true);	
    m_pMotor->SetInverted(false);
    m_pMotor->ConfigForwardSoftLimitThreshold(ElevatorConstants::k_ElevatorTopSoftLimit, 10);
    m_pMotor->ConfigReverseSoftLimitThreshold(ElevatorConstants::k_ElevatorBottomSoftLimit, 10);
    m_pMotor->ConfigForwardSoftLimitEnable(true, 10);
    m_pMotor->ConfigReverseSoftLimitEnable(false, 10);
    m_pMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
}

void Elevator::Periodic(){
    frc::SmartDashboard::PutNumber("Elevator Target Position", GetTargetPosition());
    frc::SmartDashboard::PutNumber("Elevator Actual Position", GetActualPosition());
    frc::SmartDashboard::PutBoolean("Elevator On Target", IsOnTarget());
    frc::SmartDashboard::PutNumber("Elevator Stator Current", m_pMotor->GetStatorCurrent());
    frc::SmartDashboard::PutNumber("Elevator Supply Current", m_pMotor->GetSupplyCurrent());
    // if (m_elevatorBeambreak->Get() == true) {
    //     m_pMotor->SetSelectedSensorPosition(0);
    // }

    if (IsInAllTheWay()) {
        Zero();
    }

    frc::SmartDashboard::PutBoolean("Elevator Beam Break",m_elevatorBeambreak->Get());    
}

void Elevator::Stop(){
    m_pMotor->Set(TalonFXControlMode::PercentOutput, 0);
}

void Elevator::SetTargetPosition(double pos){
    m_desiredPosition = pos;
    m_pMotor->Set(ControlMode::MotionMagic, pos);
}

void Elevator::SetPerceptOutput(double pct) {
    m_pMotor->Set(ControlMode::PercentOutput, pct);
}

double Elevator::GetTargetPosition(){
    return m_desiredPosition;
}

double Elevator::GetActualPosition(){
    return m_pMotor->GetSelectedSensorPosition();
}

void Elevator::Zero(){
    m_pMotor->SetSelectedSensorPosition(0, 0, 10);
}

bool Elevator::IsOnTarget(){
    return (abs(m_pMotor->GetActiveTrajectoryPosition() - GetTargetPosition()) < ElevatorConstants::k_ElevatorOnTargetThreshold) &&
    ((abs(GetTargetPosition() - GetActualPosition())) < (ElevatorConstants::k_ElevatorOnTargetThreshold * 2));
}

void Elevator::EngageBrake(){
    m_brakeSolenoid->Set(m_brakeSolenoid->kForward);
}

void Elevator::ReleaseBrake(){
    m_brakeSolenoid->Set(m_brakeSolenoid->kReverse);
}

bool Elevator::IsInAllTheWay(){
    static int stalled_count = 0;
    // if (m_pMotor->GetSupplyCurrent() >  frc::Preferences::GetDouble("ELEVATOR_STALLED_CURRENT", 20)) {
    if(abs(m_pMotor->GetSelectedSensorVelocity()) < 5000 && (abs(m_pMotor->GetMotorOutputPercent()) > 0.5) && (m_pMotor->GetSupplyCurrent() > 80)){
        stalled_count++;
    } else {
        stalled_count = 0;
    }
    return stalled_count > 10;
    
}