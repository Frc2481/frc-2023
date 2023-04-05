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
#include <frc2/command/ConditionalCommand.h>


class AcquireGamePieceCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
    AcquireGamePieceCommand> {

      private:
      Gripper* m_pGripper;
      Intake* m_pIntake;
      Flipper* m_pFlipper;  

 public:
  AcquireGamePieceCommand(Gripper* gripper, Intake* intake, Flipper* flipper, bool fastMode = false, bool cube = false){
    m_pGripper = gripper;
    m_pIntake = intake;
    m_pFlipper = flipper;
  
  AddRequirements({m_pGripper, m_pIntake, m_pFlipper});
  AddCommands(

    frc2::SequentialCommandGroup{

        // Make sure the gripper is open
        m_pGripper->OpenCommand(),
        
        // If the gripper is up then put it down and add an additiona 0.5 second delay to allow it to go down.
        frc2::ConditionalCommand(frc2::SequentialCommandGroup{
            m_pFlipper->DownCommand(),
            frc2::WaitCommand(0.5_s)  
          }, 
          frc2::InstantCommand([]{}),
          [this] {return m_pFlipper->IsUp();}),

        m_pFlipper->DownCommand(),
        m_pIntake->ExtendCommand(),
        // m_pIntake->TurnOnIntakeCommand(),
         frc2::InstantCommand([this, cube] {m_pIntake->TurnOnIntake(
          cube ? IntakeConstants::k_IntakeHorizontalRollerSpeedCube : IntakeConstants::k_IntakeHorizontalRollerSpeed, 
          cube ? IntakeConstants::k_IntakeVerticalRollerSpeedCube :  IntakeConstants::k_IntakeVerticalRollerSpeed);},{m_pIntake}),
        m_pIntake->WaitForGamePieceCommand(),
        m_pIntake->TurnOffHorizontalCommand(),
        frc2::WaitCommand(0.25_s),
        frc2::InstantCommand([this] {m_pIntake->TurnOnIntake(
          0, 
          0);},{m_pIntake}),
        // m_pIntake->TurnOffCommand(),
        frc2::ConditionalCommand(
          frc2::SequentialCommandGroup{
             m_pFlipper->UpCommand(cube),
             frc2::WaitCommand(0.5_s),
             m_pIntake->TurnOffVerticalCommand(),
             m_pGripper->CloseCommand(),
             m_pIntake->RetractCommand(),
             frc2::WaitCommand(0.5_s),
             m_pGripper->PickedUpConeCommand(),
             m_pFlipper->FloatCommand()
          },
          
          m_pGripper->PickedUpConeCommand(),  
          [fastMode] {return fastMode;}
        ),
          
        
        // frc2::WaitCommand(fastMode ? 1.0_s : 2.0_s),
        // m_pIntake->TurnOffCommand(),
        // m_pGripper->CloseCommand(),
        // m_pIntake->RetractCommand(),
        // frc2::WaitCommand(0.5_s),
        // m_pGripper->PickedUpConeCommand(),
        
        // m_pGripper->CloseCommand(),
        // m_pFlipper->FloatCommand()
    }
  );
  }  
};
