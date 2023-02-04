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

void Drivetrain::Periodic() {
  // Drive(units::meters_per_second_t(0.0), units::meters_per_second_t(0.0), units::radians_per_second_t(0));
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot) {
  auto states = m_kinematics.ToSwerveModuleStates(
      m_fieldCentricForJoystick ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, frc::Rotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  // m_poseEstimator.Update(m_gyro.GetRotation2d(),
  //                        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
  //                         m_backLeft.GetPosition(), m_backRight.GetPosition()});

  

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

bool Drivetrain::getFieldCentricForJoystick(){
  return m_fieldCentricForJoystick;
}

frc::SwerveDriveKinematics<4> & Drivetrain::GetKinematics(){
  return m_kinematics;
}

void Drivetrain::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_backLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_backRight.ResetEncoders();
}
