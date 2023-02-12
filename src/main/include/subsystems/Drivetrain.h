// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/field2d.h>

#include "components/SwerveModule.h"

#include "AHRS.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain : public frc2::SubsystemBase{
 public:
  Drivetrain();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot);
  void UpdateOdometry();

  bool getFieldCentricForJoystick();

  void toggleFieldCentricForJoystick();

  void ResetEncoders();

  void Periodic();

  frc::Rotation2d GetHeading();

  void ZeroHeading(double offset = 0);

  void ResetOdometry(frc::Pose2d pose);

  frc::Pose2d GetOdometryPosition();

  frc::SwerveDriveKinematics<4> & GetKinematics();

  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  frc::Field2d* GetField();

 private:
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

  SwerveModule m_frontLeft{FalconIDs::kFrontLeftDriveMotorID, 
                           FalconIDs::kFrontLeftDriveMotorFollowerID, 
                           3, 1, false, true, "FRONT_LEFT"};
  SwerveModule m_frontRight{FalconIDs::kFrontRightDriveMotorID, 
                            FalconIDs::kFrontRightDriveMotorFollowerID, 
                            6, 2, false, true, "FRONT_RIGHT"};
  SwerveModule m_backLeft{FalconIDs::kBackLeftDriveMotorID, 
                          FalconIDs::kBackLeftDriveMotorFollowerID, 
                          9, 3, false, true, "BACK_LEFT"};
  SwerveModule m_backRight{FalconIDs::kBackRightDriveMotorID, 
                           FalconIDs::kBackRightDriveMotorFollowerID, 
                           12, 4, false, true, "BACK_RIGHT"};

  AHRS m_IMU{
    frc::SPI::kMXP
  };

  double m_yawOffset = 0;


  bool m_fieldCentricForJoystick = false;

  frc::Field2d m_field;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  // Gains are for example purposes only - must be determined for your own
  // robot!
 frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
     m_kinematics,
      frc::Rotation2d{},
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      frc::Pose2d{},
      // {0.1, 0.1, 0.1},
      // {0.1, 0.1, 0.1}
      };

    // frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
    //  m_kinematics,
    //   frc::Rotation2d{},
    //   {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
    //    m_backLeft.GetPosition(), m_backRight.GetPosition()},
    //   frc::Pose2d{},
    //   {0.1, 0.1, 0.1},
    //   {0.1, 0.1, 0.1}};
};
