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
#include "commands/ScoreGamePieceCommand.h"
#include "commands/ElevatorGoToPositionCommand.h"
#include <frc2/command/ParallelCommandGroup.h>




/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LeftLaneBlueAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
    LeftLaneBlueAutoCommand> {

      private:
      Drivetrain* m_pDrive;
      Elevator* m_pElevator;
      Flipper* m_pFlipper;
      Gripper* m_pGripper;
      Intake* m_pIntake;
      Slide* m_pSlide;
      frc::Pose2d m_initialPosition;


 public:
  LeftLaneBlueAutoCommand(Drivetrain* drive, Elevator* elevator, Flipper* flipper, Gripper* gripper, Intake* intake, Slide* slide){
    m_pDrive = drive;
    m_pElevator = elevator;
    m_pFlipper = flipper;
    m_pGripper = gripper;
    m_pIntake = intake;
    m_pSlide = slide;
    m_initialPosition = frc::Pose2d{0_in, 0_in, 0_deg};

    frc::TrajectoryConfig forwardConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel)};
    forwardConfig.SetKinematics(m_pDrive->GetKinematics());
    forwardConfig.SetReversed(false);

    frc::TrajectoryConfig reverseConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel)};
    reverseConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseConfig.SetReversed(true);

    frc::TrajectoryConfig reverseChargeStationConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel / 2)};
    reverseChargeStationConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseChargeStationConfig.SetReversed(true);

    AddCommands(
      
     frc2::SequentialCommandGroup{
        
        frc2::InstantCommand([this]{m_pDrive->ResetOdometry(m_initialPosition);},{m_pDrive}),
        m_pGripper->CloseCommand(),
        ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorTopPosition), // Elevator goes out
        m_pElevator->WaitForElevatorOnTargetCommand(),
        m_pGripper->OpenCommand(),
        frc2::WaitCommand(0.25_s),

        frc2::ParallelDeadlineGroup{
           FollowPathCommand(
            m_initialPosition,
            {frc::Translation2d{135_in, -6_in}, frc::Translation2d{175_in, -14_in}},
            frc::Pose2d{200_in, -16_in, 0_deg},
            forwardConfig, m_pDrive),
          frc2::SequentialCommandGroup{
            ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorStowPosition),
            frc2::ScheduleCommand(new AcquireGamePieceCommand(m_pGripper, m_pIntake, m_pFlipper)), // geting 1st game piece
          }
        },
        m_pIntake->WaitForGamePieceCommand(),
        frc2::ParallelCommandGroup{
          FollowPathCommand(
            frc::Pose2d{200_in, -16_in, 0_deg},
            {frc::Translation2d{150_in, -18_in}, frc::Translation2d{50_in, -22_in}},
            frc::Pose2d{0_in, -24_in, 0_deg},
            reverseConfig, m_pDrive),
          frc2::SequentialCommandGroup{
            m_pGripper->WaitForGamePieceCommand(),
            ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorTopPosition), // Elevator goes out
        }
        },
        m_pElevator->WaitForElevatorOnTargetCommand(),
        m_pGripper->OpenCommand(),
        frc2::WaitCommand(0.5_s),
        m_pGripper->DroppedGamePieceCommand(),

        frc2::ParallelDeadlineGroup{
          FollowPathCommand(
            frc::Pose2d{0_in, -24_in, 0_deg},
            {frc::Translation2d{135_in, -30_in}, frc::Translation2d{175_in, -40_in}},
            frc::Pose2d{200_in, -60_in, 0_deg},
            forwardConfig, m_pDrive),
            ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorStowPosition),
        },

        FollowPathCommand(
          frc::Pose2d{200_in, -60_in, 0_deg},
          {frc::Translation2d{160_in, -65_in}, frc::Translation2d{120_in, -75_in}},
          frc::Pose2d{65_in, -85_in, 0_deg},
          reverseChargeStationConfig, m_pDrive),

      

          

        // FollowPathCommand(
        //   frc::Pose2d{24_in, 5_in, 0_deg}, 
        //   {frc::Translation2d{16_in, 3_in}, frc::Translation2d{8_in, 1_in}},

        //   frc::Pose2d{0_in, 0_in, 0_deg},
        //   reverseConfig, m_pDrive)  
  });

            // Balance???
      }
  };

