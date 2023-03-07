#include "subsystems/Elevator.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/InstantCommand.h>

frc2::InstantCommand Elevator::StowCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorStowPosition);
}

frc2::InstantCommand Elevator::GoToFloorCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorFloorPosition);
}

frc2::InstantCommand Elevator::GoToBottomPostCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorBottomPostPosition);
}

frc2::InstantCommand Elevator::GoToTopPostCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorTopPostPosition);
}

frc2::InstantCommand Elevator::GoToMidShelfCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorMidShelfPosition);
}

frc2::InstantCommand Elevator::GoToTopShelfCommand(){
    return GoToPositionCommand(ElevatorConstants::k_ElevatorTopShelfPosition);
}

frc2::InstantCommand Elevator::GoToPositionCommand(double pos){
    return frc2::InstantCommand([this, pos] {SetTargetPosition(pos);},{this});
}

frc2::WaitUntilCommand Elevator::WaitForElevatorOnTargetCommand(){
    return frc2::WaitUntilCommand([this] {return IsOnTarget();});
}

frc2::InstantCommand Elevator::EngageBrakeCommand(){
    return frc2::InstantCommand([this] {EngageBrake();},{this});
}

frc2::InstantCommand Elevator::ReleaseBrakeCommand(){
    return frc2::InstantCommand([this] {ReleaseBrake();},{this});
}

Elevator::Elevator(){

     m_brakeSolenoid = new frc::DoubleSolenoid(
        frc::PneumaticsModuleType::REVPH, 
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
    m_pMotor->ConfigReverseSoftLimitEnable(true, 10);
    m_pMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
}

void Elevator::Periodic(){
    frc::SmartDashboard::PutNumber("Elevator Target Position", GetTargetPosition());
    frc::SmartDashboard::PutNumber("Elevator Actual Position", GetActualPosition());
    frc::SmartDashboard::PutBoolean("Elevator On Target", IsOnTarget());
    // if (m_elevatorBeambreak->Get() == true) {
    //     m_pMotor->SetSelectedSensorPosition(0);
    // }
    frc::SmartDashboard::PutBoolean("Elevator Beam Break",m_elevatorBeambreak->Get());
}

void Elevator::Stop(){
    m_pMotor->Set(TalonFXControlMode::PercentOutput, 0);
}

void Elevator::SetTargetPosition(double pos){
    m_desiredPosition = pos;
    m_pMotor->Set(ControlMode::MotionMagic, pos);
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
    return abs(m_pMotor->GetActiveTrajectoryPosition() - GetTargetPosition()) < ElevatorConstants::k_ElevatorOnTargetThreshold;
}

void Elevator::EngageBrake(){
    m_brakeSolenoid->Set(m_brakeSolenoid->kForward);
}

void Elevator::ReleaseBrake(){
    m_brakeSolenoid->Set(m_brakeSolenoid->kReverse);
}

bool Elevator::IsInAllTheWay(){
    return m_elevatorBeambreak->Get();
}