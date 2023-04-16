// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "RobotParameters.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Flipper.h"
#include "subsystems/Gripper.h"
#include "subsystems/Intake.h"
#include "subsystems/Slide.h"
#include <frc2/command/ParallelDeadlineGroup.h>
#include "commands/FollowPathCommand.h"
#include "commands/AcquireGamePieceCommand.h"
#include "commands/ScoreGamePieceCommand.h"
#include "commands/WaitForPitchCommand.h"
#include <frc2/command/InstantCommand.h>
#include "commands/ElevatorGoToPositionCommand.h" 



/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class CenterLaneBlueAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
             CenterLaneBlueAutoCommand> {

      private:
      Drivetrain* m_pDrive;
      Elevator* m_pElevator;
      Flipper* m_pFlipper;
      Gripper* m_pGripper;
      Intake* m_pIntake;
      Slide* m_pSlide;
      frc::Pose2d m_initialPosition;

 public:
  CenterLaneBlueAutoCommand(Drivetrain* drive, Elevator* elevator, Flipper* flipper, Gripper* gripper, Intake* intake, Slide* slide){
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

    AddCommands(

      frc2::SequentialCommandGroup{
          frc2::InstantCommand([this]{m_pDrive->ResetOdometry(m_initialPosition);},{m_pDrive}),
          m_pGripper->CloseCommand(),
          frc2::ScheduleCommand(new ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorTopPosition, true)), // Elevator goes out
          m_pFlipper->DownCommand(),
          m_pElevator->WaitForElevatorPastPositionCommand(),
          m_pGripper->OpenCommand(),
          m_pGripper->DroppedGamePieceCommand(),
          frc2::WaitCommand(0.5_s),
          frc2::ScheduleCommand(new ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorStowPosition)),
         
          frc2::InstantCommand([this] {return m_pIntake->Extend();}),
          frc2::WaitCommand(0.5_s),

          frc2::InstantCommand([this]{m_pDrive->Drive(1.5_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}),
          WaitForPitchCommand(m_pDrive, 10),
          frc2::InstantCommand([this] {return m_pIntake->Retract();}),
          frc2::InstantCommand([this]{m_pDrive->Drive(1.5_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}),
          WaitForPitchCommand(m_pDrive, -9),
          frc2::InstantCommand([this]{m_pDrive->Drive(1.5_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}),
          WaitForPitchCommand(m_pDrive, -6),
          frc2::WaitCommand(0.1_s),
          frc2::InstantCommand([this]{m_pDrive->Drive(0_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}),
          
          // frc2::InstantCommand([this]{m_pDrive->ResetOdometry(m_initialPosition);},{m_pDrive}),

          frc2::InstantCommand([this]{m_pDrive->Drive(1.5_mps, 0_mps, 4_rad_per_s);}, {m_pDrive}),
          frc2::WaitCommand(0.45_s),
          //  FollowPathCommand(
          //   frc::Pose2d{230_in, 0_in, 0_deg},
          //   {},
          //   frc::Pose2d{200_in, 0_in, 180_deg},
          //   reverseConfig, m_pDrive),

          frc2::InstantCommand([this] {return m_pIntake->Extend();}),
          frc2::WaitCommand(0.25_s),

          frc2::InstantCommand([this]{m_pDrive->Drive(1.5_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}),
          WaitForPitchCommand(m_pDrive, 13),
          frc2::InstantCommand([this]{m_pDrive->Drive(0.5_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}), 
          WaitForPitchCommand(m_pDrive, 11),
          frc2::InstantCommand([this]{m_pDrive->Drive(0.5_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}), //back up a little 
          WaitForPitchCommand(m_pDrive, 1),
          frc2::InstantCommand([this]{m_pDrive->Drive(0_mps, -0.1_mps, 0_deg_per_s);}, {m_pDrive}), //back up a little 
          frc2::WaitCommand(0.25_s),
          frc2::InstantCommand([this]{m_pDrive->Drive(0_mps, 0_mps, 0_deg_per_s);}, {m_pDrive}),

      }
    );
  }
  };
