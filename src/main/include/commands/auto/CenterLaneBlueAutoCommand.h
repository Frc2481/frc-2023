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

#include "commands/AcquireGamePieceCommand.h"
#include "commands/ScoreGamePieceCommand.h"


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

 public:
  CenterLaneBlueAutoCommand(Drivetrain* drive, Elevator* elevator, Flipper* flipper, Gripper* gripper, Intake* intake, Slide* slide){
    m_pDrive = drive;
    m_pElevator = elevator;
    m_pFlipper = flipper;
    m_pGripper = gripper;
    m_pIntake = intake;
    m_pSlide = slide;

    AddCommands(

      frc2::SequentialCommandGroup{
        m_pElevator->GoToTopPostCommand(),
        
        // ScoreGamePieceCommand(), //add needed subsystems

      //   // Retract Elevator??

      //   // DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false),

        m_pIntake->ExtendCommand(),
        m_pIntake->TurnOnIntakeCommand(),
        // Wait For Game Piece Command??
        m_pIntake->RetractCommand(),
        
      // //Acqiure Game Piece Command?

      //   // DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false),

      //   // Balance??
        
      }

    );
  }
  };
