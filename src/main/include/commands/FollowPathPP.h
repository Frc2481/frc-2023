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
#include "RobotParameters.h"
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/DriveTrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowPathPPCommand
    : public frc2::CommandHelper<frc2::CommandBase, FollowPathPPCommand> {

 private:
    frc::Trajectory m_trajectory;
    frc::Translation2d m_pathTranslation;
    Drivetrain* m_drivetrain;

    int m_pathIdx;

 std::vector<frc::Trajectory::State> m_path;

 public:
  FollowPathPPCommand(const frc::Pose2d & start, 
                const std::vector<frc::Translation2d> & innerWayPoints,
                const frc::Pose2d & end,
                const frc::TrajectoryConfig & config, 
                Drivetrain* drivetrain,
                frc::Translation2d pathTranslation = frc::Translation2d()) {
    
    m_pathTranslation = pathTranslation;
    m_drivetrain = drivetrain;
    
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(start, innerWayPoints, end, config);
    m_trajectory = m_trajectory.TransformBy(frc::Transform2d(m_pathTranslation, frc::Rotation2d()));    
  }

      
  void Initialize() {
    m_pathIdx = 0;
  }

  void Execute() {
    
    // TODO: Get current location of robot
    frc::Pose2d pose = m_drivetrain->GetOdometryPosition();

    
    // Find the closest point on the path.
    units::meter_t minDistance = 9999_m;
    int minIdx = -1;
    int driftingAway = 0;

    for (int searchIdx = m_pathIdx; searchIdx < m_trajectory.States().size(); ++searchIdx) {
      frc::Pose2d pathPose = m_trajectory.States()[searchIdx].pose;

      units::meter_t distToPathPoint = pathPose.RelativeTo(pose).Translation().Norm();

      if (distToPathPoint < minDistance) {
        minDistance = distToPathPoint;
        minIdx = searchIdx;
        driftingAway = 0;
      } else {
        driftingAway++; 
        if (driftingAway > 10) {
          break;
        }      
      }
    }
    frc::Trajectory::State state = m_trajectory.States()[minIdx];

    // TODO: Base this on velocity of robot.
    units::meter_t lookAheadDistance = 0.25_m;
    int lookAheadIdx;

    for (lookAheadIdx = minIdx; minIdx < m_trajectory.States().size(); ++lookAheadIdx) {
      frc::Pose2d pathPose = m_trajectory.States()[lookAheadIdx].pose;

      units::meter_t distToPathPoint = pathPose.RelativeTo(pose).Translation().Norm();

      if (distToPathPoint > lookAheadDistance) {
        break;
      }
    }

    frc::Trajectory::State state = m_trajectory.States()[minIdx];

    frc::Trajectory::State lookAheadState = m_trajectory.States()[lookAheadIdx];
    frc::Pose2d lookAheadPose = state.pose;

    frc::Translation2d driveVector = pose.RelativeTo(lookAheadPose).Translation();
    frc::Rotation2d driveAngle = driveVector.Angle();

    units::meters_per_second_t xVel = driveAngle.Cos() * state.velocity;
    units::meters_per_second_t yVel = driveAngle.Sin() * state.velocity;

    

    // TODO: Skip ahead look ahead distance.

    // TODO: Correct for any cross track error.

    
  }

  void End(bool interrupted) {

  }

  bool IsFinished() {

  }
};
