#include "subsystems/Elevator.h"
#include "RobotParameters.h"


Elevator::Elevator(){
    m_pMotor = new TalonFX(FalconIDs::kElevatorMotor);
    m_pMotor->ConfigFactoryDefault();
    m_pMotor->Config_kP(0, RobotParameters::k_steerMotorControllerKp, 10);
    m_pMotor->Config_kI(0, RobotParameters::k_steerMotorControllerKi, 10);
    m_pMotor->Config_kD(0, RobotParameters::k_steerMotorControllerKd, 10);
    m_pMotor->Config_kF(0, RobotParameters::k_steerMotorControllerKd, 10);
    m_pMotor->Config_IntegralZone(0, 0, 10);
    m_pMotor->ConfigMaxIntegralAccumulator (0, 0, 10);
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
}

void Elevator::Periodic(){

}

void Elevator::SetDesiredPosition(double pos){

}

double Elevator::GetDesiredPosition(){

}

double Elevator::GetActualPosition(){

}

void Elevator::Zero(){

}

bool Elevator::IsOnTarget(){

}