// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/Timer.h>
#include "ExampleGlobalMeasurementSensor.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTableInstance.h"
#include "units/length.h"
#include <frc/geometry/Rotation2d.h>
#include "Utils/NormalizeToRange.h"

void Drivetrain::Periodic() {
  // Drive(units::meters_per_second_t(0.0), units::meters_per_second_t(0.0), units::radians_per_second_t(0));
  frc::SmartDashboard::PutNumber("IMU heading", (double)GetHeading().Degrees());
  UpdateOdometry();
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot) {
  auto states = m_kinematics.ToSwerveModuleStates(
      m_fieldCentricForJoystick ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  frc::SmartDashboard::PutNumber("fl angle", (double)fl.angle.Degrees());

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_poseEstimator.Update(GetHeading(),
                         {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                          m_backLeft.GetPosition(), m_backRight.GetPosition()});

  frc::SmartDashboard::PutNumber("Odometry X", m_poseEstimator.GetEstimatedPosition().X().to<double>());
  frc::SmartDashboard::PutNumber("Odometry Y", m_poseEstimator.GetEstimatedPosition().Y().to<double>());
  frc::SmartDashboard::PutNumber("Odometry Yaw", m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().to<double>());

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  // bot pose type
  if(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0)) {
      std::vector<double> default_bot_pose = {0, 0, 0};
      std::vector<double> bot_pose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose", default_bot_pose);
      frc::Pose2d global_pose{units::meter_t(bot_pose[0]), units::meter_t(bot_pose[0]), 0_deg}; 

      // m_poseEstimator.AddVisionMeasurement(
        // global_pose, 
        // frc::Timer::GetFPGATimestamp() - 0.3_s);
        
  }
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  ZeroHeading();
  m_poseEstimator.ResetPosition(GetHeading(), 
                        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                         m_backLeft.GetPosition(), m_backRight.GetPosition()}, pose);
  
}

frc::Pose2d Drivetrain::GetOdometryPosition(){
  return m_poseEstimator.GetEstimatedPosition();
}

bool Drivetrain::getFieldCentricForJoystick(){
  return m_fieldCentricForJoystick;
}

frc::Rotation2d Drivetrain::GetHeading(){
  return frc::Rotation2d(units::degree_t(-normalizeToRange::NormalizeToRange(m_IMU.GetYaw() - m_yawOffset, -180, 180, true)));
}

void Drivetrain::ZeroHeading(double offset){
  m_IMU.Reset();
  m_yawOffset = offset;
}

frc::SwerveDriveKinematics<4> & Drivetrain::GetKinematics(){
  return m_kinematics;
}

void Drivetrain::toggleFieldCentricForJoystick(){
  m_fieldCentricForJoystick = !m_fieldCentricForJoystick; 
  frc::SmartDashboard::PutBoolean("fieldCentric", m_fieldCentricForJoystick);
}

void Drivetrain::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_backLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_backRight.ResetEncoders();
}
