// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Flipper.h"
#include "subsystems/Gripper.h"
#include "subsystems/Intake.h"
#include "subsystems/Slide.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/InstantCommand.h>
#include "RobotParameters.h"
#include "commands/ScoreGamePieceCommand.h"
#include "commands/FollowPathCommand.h"
#include <frc2/command/ParallelDeadlineGroup.h>
#include "commands/AcquireGamePieceCommand.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RightLaneRedAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
    RightLaneRedAutoCommand> {

      private:
      Drivetrain* m_pDrive;
      Elevator* m_pElevator;
      Flipper* m_pFlipper;
      Gripper* m_pGripper;
      Intake* m_pIntake;
      Slide* m_pSlide;

 public:
  RightLaneRedAutoCommand(Drivetrain* drive, Elevator* elevator, Flipper* flipper, Gripper* gripper, Intake* intake, Slide* slide){
    m_pDrive = drive;
    m_pElevator = elevator;
    m_pFlipper = flipper;
    m_pGripper = gripper;
    m_pIntake = intake;
    m_pSlide = slide;
    frc::TrajectoryConfig forwardConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel)};
    forwardConfig.SetKinematics(m_pDrive->GetKinematics());
    forwardConfig.SetReversed(false);

    frc::TrajectoryConfig reverseConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel)};
    reverseConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseConfig.SetReversed(true);

    AddCommands(
      
      frc2::SequentialCommandGroup{
        ScoreGamePieceCommand(TOP, m_pElevator, m_pGripper, m_pSlide),
        frc2::ParallelDeadlineGroup(
          std::move(AcquireGamePieceCommand(m_pGripper, m_pIntake, m_pFlipper)),
          std::move(FollowPathCommand(
            frc::Pose2d{0_in, 0_in, 0_deg}, 
            {frc::Translation2d{0_in, 0_in}, frc::Translation2d{0_in, 0_in}},
            frc::Pose2d{0_in, 0_in, 0_deg},
            forwardConfig, m_pDrive)
          )
        ),
        FollowPathCommand(
            frc::Pose2d{0_in, 0_in, 0_deg}, 
            {frc::Translation2d{0_in, 0_in}, frc::Translation2d{0_in, 0_in}},
            frc::Pose2d{0_in, 0_in, 0_deg},
            reverseConfig, m_pDrive),
        //TODO figure out what/where scoring
        ScoreGamePieceCommand(TOP, m_pElevator, m_pGripper, m_pSlide),
        frc2::ParallelDeadlineGroup(
          std::move(AcquireGamePieceCommand(m_pGripper, m_pIntake, m_pFlipper)),
          std::move(FollowPathCommand(
            frc::Pose2d{0_in, 0_in, 0_deg}, 
            {frc::Translation2d{0_in, 0_in}, frc::Translation2d{0_in, 0_in}},
            frc::Pose2d{0_in, 0_in, 0_deg},
            forwardConfig, m_pDrive)
          )
        ),
        FollowPathCommand(
            frc::Pose2d{0_in, 0_in, 0_deg}, 
            {frc::Translation2d{0_in, 0_in}, frc::Translation2d{0_in, 0_in}},
            frc::Pose2d{0_in, 0_in, 0_deg},
            reverseConfig, m_pDrive),
        //TODO figure out what/where scoring
        ScoreGamePieceCommand(TOP, m_pElevator, m_pGripper, m_pSlide)
      }
    );
  }
};
