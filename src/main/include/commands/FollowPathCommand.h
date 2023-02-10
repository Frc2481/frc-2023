// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/HolonomicDriveController.h>

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
      frc::ProfiledPIDController<units::radians> m_yawController;
      frc2::PIDController m_XController;
      frc2::PIDController m_YController;
      frc::HolonomicDriveController m_swerveController{m_XController, m_YController, m_yawController};

 public:
  FollowPathCommand(const frc::Pose2d & start, 
                    const std::vector<frc::Translation2d> & innerWayPoints,
                    const frc::Pose2d & end,
                    const frc::TrajectoryConfig & config){

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(start, innerWayPoints, end, config);
  }

  void Initialize() override{}

  void Execute() override{}

  void End(bool interrupted) override{}

  bool IsFinished() override{
    return false;
  }
};
