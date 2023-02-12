/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "components/SwerveModule.h"
#include "components/CTRECANEncoder.h"
#include <frc/geometry/Rotation2d.h>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
#include "components/MotorPositionController.h"
#include "utils/NormalizeToRange.h"

// using namespace ctre::phoenix::motorcontrol;
/*
void SetTalonConfig(TalonConfig config) {
  m_motor->GetPIDController()->SetP(talonConfig.slo0.kp);
  m_motor->GetPIDController()->SetI(talonConfig.slo0.ki);
  m_motor->GetPIDController()->SetD(talonConfig.slo0.kd);
}
*/


SwerveModule::SwerveModule(int driveMotorID, int driveMotorFollowerID, int turningMotorID, int turnEncoderID,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed, const std::string &name) :m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed),  
      m_name(name){
      m_pDriveMotor = new TalonFX(driveMotorID);
      m_pTurningMotor = new TalonSRX(turningMotorID);
      // m_pTurningMotor = new VictorSPX(turningMotorID);
      
      m_pDriveMotor->ConfigFactoryDefault();
      m_pTurningMotor->ConfigFactoryDefault();
      // m_pTurningEncoder = new CTRECANEncoder(turnEncoderID, name);
      m_pTurningEncoder = new CTREMagEncoder(m_pTurningMotor, name);
      // m_pTurningMotor->ConfigRemoteFeedbackFilter(*(m_pTurningEncoder->getCANCoder()), 0);
      // m_pTurningMotor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0, 0, 10);
      
      m_pTurningMotor->SelectProfileSlot(0, 0);
	
      m_pTurningMotor->Config_kP(0, RobotParameters::k_steerMotorControllerKp, 10);
      m_pTurningMotor->Config_kI(0, RobotParameters::k_steerMotorControllerKi, 10);
      m_pTurningMotor->Config_kD(0, RobotParameters::k_steerMotorControllerKd, 10);
      m_pTurningMotor->Config_IntegralZone(0, 0, 10);
      m_pTurningMotor->ConfigMaxIntegralAccumulator (0, 0, 10);
      m_pTurningMotor->SetNeutralMode(NeutralMode::Brake);
      m_pTurningMotor->EnableVoltageCompensation(true);
      m_pTurningMotor->ConfigVoltageCompSaturation(12.0, 0);
      m_pTurningMotor->ConfigNeutralDeadband(0.04, 0);
      m_pTurningMotor->ConfigNominalOutputForward(0.0, 0.0);
      m_pTurningMotor->ConfigNominalOutputReverse(0.0, 0.0);
      m_pTurningMotor->ConfigPeakOutputForward(1.0, 0.0);
      m_pTurningMotor->ConfigPeakOutputReverse(-1.0, 0.0);
      m_pTurningMotor->SetSensorPhase(turningEncoderReversed);	
      m_pTurningMotor->SetInverted(false);


      
      m_pDriveMotor->ConfigFactoryDefault();
      m_pDriveMotor->SetInverted(driveEncoderReversed);
      m_pDriveMotor->Config_kP(0, RobotParameters::k_driveMotorControllerKp);//.07
      m_pDriveMotor->Config_kI(0, RobotParameters::k_driveMotorControllerKi);
      m_pDriveMotor->Config_kD(0, RobotParameters::k_driveMotorControllerKd);//.035
      m_pDriveMotor->Config_kF(0, 1023/(RobotParameters::k_maxSpeed.value()/RobotParameters::k_driveMotorEncoderTicksToMPS));
      m_pDriveMotor->Config_IntegralZone(0, 0);
      m_pDriveMotor->ConfigOpenloopRamp(0.5, 10);
      m_pDriveMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
      m_pDriveMotor->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 20, 0);
  // // Set the distance per pulse for the drive encoder. We can simply use the
  // // distance traveled for one rotation of the wheel divided by the encoder
  // // resolution.
  // m_driveEncoder.SetDistancePerPulse(
  //     ModuleConstants::kDriveEncoderDistancePerPulse);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
  // divided by the encoder resolution.

  // m_turningEncoder.SetDistancePerPulse(
  //     ModuleConstants::kTurningEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(units::radian_t(-std::numbers::pi),
                                               units::radian_t(std::numbers::pi));
  
}

frc::SwerveModuleState SwerveModule::GetState() const {
    // TODO: acount for gear reduction
  return {units::meters_per_second_t{m_pDriveMotor->GetSelectedSensorVelocity()},//MATH_CONSTANTS_PI
          frc::Rotation2d(units::degree_t(m_pTurningEncoder->getAngle()))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const{
    // TODO: acount for gear reduction
return {units::meter_t{m_pDriveMotor->GetSelectedSensorPosition() * RobotParametersCompetition::k_driveMotorEncoderTicksToMeters},//MATH_CONSTANTS_PI
          frc::Rotation2d(units::degree_t(m_pTurningEncoder->getAngle()))};

}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
  m_pTurningEncoder->update();
  
  // Calculate the drive output from the drive PID controller.
  // const auto driveOutput = m_drivePIDController.Calculate(
  //     m_driveMotor->GetEncoder().GetVelocity(), state.speed.to<double>());

  float currentAngle = units::degree_t(m_pTurningEncoder->getAngle()).to<double>();
  float driveMotorRPM = state.speed.to<double>() * RobotParameters::k_driveMotorEncoderTicksToMPS;
  float desiredAngle = state.angle.Degrees().to<double>();

  // // Calculate the turning motor output from the turning PID controller.
  // auto turnOutput = m_turningPIDController.Calculate(
  //     units::radian_t(units::degree_t(m_turningEncoder->getAngle())), state.angle.Radians());
    
    frc::SmartDashboard::PutNumber("Desired", state.angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Current", units::degree_t(m_pTurningEncoder->getAngle()).to<double>());
  //   frc::SmartDashboard::PutNumber("Turn Output", turnOutput); //TODO: These shouldn't stay here
  //   frc::SmartDashboard::PutNumber("Speed", m_driveMotor->GetEncoder().GetVelocity());
      // printf("%s: current angle %f desired angle %f\n", 
      //     m_name.c_str(), 
      //     units::radian_t(m_turningEncoder->getAngle()).to<double>(), 
      //     state.angle.Radians().to<double>());
  // printf("\nC A: %f\n", currentAngle); //TODO: Remove these printfs once turn issue fixed
  // printf("\nD A: %f\n", desiredAngle);

  if(fabs(normalizeToRange::RangedDifference(currentAngle - desiredAngle, -180, 180)) > 90){//used to be 90
    desiredAngle = normalizeToRange::NormalizeToRange(desiredAngle+180, -180, 180, true);
    driveMotorRPM = driveMotorRPM * -1;
  }
  if(fabs(driveMotorRPM) < 0.01){//TODO find better zone
    desiredAngle = currentAngle;
    // driveMotorRPM= 0;
    // printf("\nD%f\n", driveMotorRPM);
  }

  // printf("\nUpdated Current Angle: %f\n", currentAngle);
  // printf("\nUpdated Desired Angle: %f\n", desiredAngle);
  // printf("\nUpdated Desired Speed: %f\n", driveMotorRPM);

  // Set the motor outputs.
  // TODO Convert RPM to ticks
  m_pDriveMotor->Set(ControlMode::Velocity, driveMotorRPM);
  // m_pDriveMotor->Set(ControlMode::PercentOutput, driveMotorRPM);
  
  // m_turningMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, turnOutput);

  double desiredTicks = m_pTurningEncoder->convertAngleToTickSetpoint(desiredAngle);
  m_pTurningMotor->Set(ControlMode::Position, desiredTicks);

  
}

void SwerveModule::ResetEncoders() {
  m_pDriveMotor->SetSelectedSensorPosition(0);
  m_pTurningEncoder->zero();

}

void SwerveModule::updateSteerPID(double p, double i, double d){
  // m_turningPIDController.SetPID(p, i, d);
  // printf("Steer P: %0.1f, I: %0.1f, D: %0.1f", p, i, d);
  m_turningPIDController.SetP(p);
  m_turningPIDController.SetI(i);
  m_turningPIDController.SetD(d);
}

void SwerveModule::updateDrivePID(double p, double i, double d, double f){
  // printf("Drive P: %0.1f, I: %0.1f, D: %0.1f", p, i, d);
  // m_drivePIDController.SetPID(p, i, d);
  m_pDriveMotor->Config_kP(0,p);
  m_pDriveMotor->Config_kI(0,i);
  m_pDriveMotor->Config_kD(0,d);
  m_pDriveMotor->Config_kF(0,f);
}
void SwerveModule::setCoast(){
  m_pDriveMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void SwerveModule::setBrake(){
  m_pDriveMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void SwerveModule::DriveArc(double arcLength, double wheelAngle){
  double desiredTicks = m_pTurningEncoder->convertAngleToTickSetpoint(wheelAngle);
  m_pTurningMotor->Set(ControlMode::Position, desiredTicks);
  m_pDriveMotor->ConfigMotionCruiseVelocity((units::meters_per_second_t(RobotParameters::k_maxSpeed).value())/RobotParameters::k_driveMotorEncoderTicksToMPS);
  m_pDriveMotor->ConfigMotionAcceleration((units::meters_per_second_t(RobotParameters::k_maxSpeed).value()/RobotParameters::k_driveMotorEncoderTicksToMPS)*2);
  m_pDriveMotor->Set(ControlMode::MotionMagic, m_pDriveMotor->GetSelectedSensorPosition() + arcLength/RobotParameters::k_driveMotorEncoderTicksToMeters);

}

void SwerveModule::SyncCANcoders(){
  // m_pTurningEncoder->getCANCoder()->SetPositionToAbsolute();
}
