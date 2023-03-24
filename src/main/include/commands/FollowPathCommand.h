// Copyright (c) FIRST and other WPILib contributors.
// Open Source So(ftware; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/HolonomicDriveController.h>
#include "RobotParameters.h"
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowPathCommand
    : public frc2::CommandHelper<frc2::CommandBase, FollowPathCommand> {

      private:
      frc::Trajectory m_trajectory;
      frc::ProfiledPIDController<units::radians> m_yawController{RobotParameters::k_yawKp, RobotParameters::k_yawKi, RobotParameters::k_yawKd, frc::ProfiledPIDController<units::radians>::Constraints(RobotParameters::k_maxYawRate, RobotParameters::k_maxYawAccel)};
      frc2::PIDController m_XController{RobotParameters::k_xyKp, RobotParameters::k_xyKi, RobotParameters::k_xyKd};
      frc2::PIDController m_YController{RobotParameters::k_xyKp, RobotParameters::k_xyKi, RobotParameters::k_xyKd};
      frc::HolonomicDriveController m_swerveController{m_XController, m_YController, m_yawController};
      frc::Timer m_timer;
      units::second_t m_PrevTime;
      Drivetrain* m_drivetrain;


 public:
  FollowPathCommand(const frc::Pose2d & start, 
                    const std::vector<frc::Translation2d> & innerWayPoints,
                    const frc::Pose2d & end,
                    const frc::TrajectoryConfig & config, 
                    Drivetrain* drivetrain){
    m_drivetrain = drivetrain;
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(start, innerWayPoints, end, config);
    AddRequirements(m_drivetrain);
  }

  void Initialize() override{
    m_timer.Reset();
    m_timer.Start();
    m_drivetrain->GetField()->GetObject("traj")->SetTrajectory(m_trajectory);
  }

  void Execute() override{
    auto curTime = m_timer.Get();
    auto desiredState = m_trajectory.Sample(curTime);
    frc::SmartDashboard::PutNumber("Path X", units::inch_t(desiredState.pose.X()).value());
    frc::SmartDashboard::PutNumber("Path Y", units::inch_t(desiredState.pose.Y()).value());
    frc::SmartDashboard::PutNumber("Path Yaw", units::degree_t(m_yawController.GetSetpoint().position).value());
    frc::SmartDashboard::PutNumber("Path Velocity X", units::feet_per_second_t(desiredState.velocity).value() * desiredState.pose.Rotation().Cos());
    frc::SmartDashboard::PutNumber("Path Velocity Y", units::feet_per_second_t(desiredState.velocity).value() * desiredState.pose.Rotation().Sin());
    auto desiredRotation = m_trajectory.States().back().pose.Rotation();
    auto targetChassisSpeeds = m_swerveController.Calculate(
      m_drivetrain->GetOdometryPosition(),
      desiredState.pose, 
      desiredState.velocity, 
      desiredRotation);
    auto targetModuleStates = m_drivetrain->GetKinematics().ToSwerveModuleStates(targetChassisSpeeds);
    m_drivetrain->SetModuleStates(targetModuleStates);
    frc::SmartDashboard::PutNumber("Path Error X", units::inch_t(m_drivetrain->GetOdometryPosition().X() - desiredState.pose.X()).value());
    frc::SmartDashboard::PutNumber("Path Error Y", units::inch_t(m_drivetrain->GetOdometryPosition().Y() - desiredState.pose.Y()).value());
    frc::SmartDashboard::PutNumber("Chassis Speed X", units::feet_per_second_t(targetChassisSpeeds.vx).value());
    frc::SmartDashboard::PutNumber("Module Speed X", units::feet_per_second_t(targetModuleStates[0].speed).value());
  }

  void End(bool interrupted) override{
    m_timer.Stop();
    m_drivetrain->Drive(0_mps, 0_mps, units::radians_per_second_t(0));
  }

  bool IsFinished() override{
    return m_timer.HasElapsed(m_trajectory.TotalTime());
  }
};
