// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/Intake.h"
#include "subsystems/Flipper.h"
#include "subsystems/Gripper.h"
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/InstantCommand.h>
#include "RobotParameters.h"
#include <frc2/command/WaitCommand.h>


class AcquireGamePiece
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
    AcquireGamePiece> {

      private:
      Gripper* m_pGripper;
      Intake* m_pIntake;
      Flipper* m_pFlipper;  

 public:
  AcquireGamePiece(Gripper* gripper, Intake* intake, Flipper* flipper){
    m_pGripper = gripper;
    m_pIntake = intake;
    m_pFlipper = flipper;

  AddCommands(

    frc2::SequentialCommandGroup{
        m_pIntake->ExtendCommand(),
        m_pIntake->TurnOnIntakeCommand(),
        //wait for game piece command??
        m_pIntake->RetractCommand(),
        m_pFlipper->UpCommand(),
        m_pGripper->CloseCommand(),
        m_pFlipper->DownCommand()
    }
  );
  }  
};
