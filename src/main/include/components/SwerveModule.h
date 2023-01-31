/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
// #include <frc/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <numbers>

#include "RobotParameters.h"
#include "components/CTRECANEncoder.h"
#include "components/CTREMagEncoder.h"


class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorID, int driveMotorFollowerID, int turningMotorID, int turnEncoderID,
               bool driveEncoderReversed, bool turningEncoderReversed, const std::string &name);

//   frc::SwerveModuleState GetState();

//   void SetDesiredState(frc::SwerveModuleState& state, bool percentMode);

   frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();
    void updateSteerPID(double p, double i, double d);
    void updateDrivePID(double p, double i, double d, double f);
    void setCoast();
    void setBrake();
    void DriveArc(double arcLength, double wheelAngle);
    void SyncCANcoders();
 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(3.0 * 2.0*std::numbers::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              std::numbers::pi * 2.0 * 12.0);  // radians per second squared

  TalonFX* m_pDriveMotor;
  TalonSRX* m_pTurningMotor;
  CTREMagEncoder* m_pTurningEncoder;
//   CTRECANEncoder* m_pTurningEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;
  std::string m_name;

  frc2::PIDController m_drivePIDController{
      0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.05,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
      
    
};
