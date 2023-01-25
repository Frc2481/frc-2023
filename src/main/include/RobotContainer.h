// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "components/Joystick2481.h"
#include <frc2/command/button/Button.h>
#include <frc2/command/button/POVButton.h>
#include "components/XboxController2481.h"
#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Flipper.h"
#include "subsystems/Gripper.h"
#include "subsystems/Intake.h"
#include "subsystems/Slide.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
// class RobotContainer {
//   public:
//     RobotContainer();

//     frc2::Command* GetAutonomousCommand();
  
//   private:
//   Joystick2481 m_driverController;
//   Joystick2481 m_auxController;

//   public:
//     frc::SendableChooser<frc2::Command*> m_chooser;
//     Drivetrain m_drivetrain;
//     Elevator m_elevator;
//     Flipper m_flipper;
//     Gripper m_gripper;
//     Intake m_intake;
//     Slide m_slide;
//     // frc::PneumaticsControlModule m_pcm;
//     //driver
//     frc2::Button m_startDriver{[&] { return m_driverController.GetRawButton(XBOX_START_BUTTON); }};//
//     frc2::Button m_backDriver{[&] { return m_driverController.GetRawButton(XBOX_BACK_BUTTON); }};//

//     frc2::Button m_aButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_A_BUTTON); }};
//     frc2::Button m_bButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_B_BUTTON); }};
//     frc2::Button m_yButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_Y_BUTTON); }};
//     frc2::Button m_xButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_X_BUTTON); }};

//     frc2::Button m_rBumperDriver{[&] { return m_driverController.GetRawButton(XBOX_RIGHT_BUMPER); }};//
//     frc2::Button m_lBumperDriver{[&] { return m_driverController.GetRawButton(XBOX_LEFT_BUMPER); }};//
//     frc2::Button m_rTriggerDriver{[&] { return m_driverController.GetAxis(XBOX_RIGHT_TRIGGER, .5); }};//
//     frc2::Button m_lTriggerDriver{[&] { return m_driverController.GetAxis(XBOX_LEFT_TRIGGER, .5); }};//
    

//     //operator
//     frc2::Button m_startBackAux{[&] { return m_auxController.GetRawButton(XBOX_START_BUTTON) && m_auxController.GetRawButton(XBOX_BACK_BUTTON); }};
//     frc2::Button m_startAux{[&] { return m_auxController.GetRawButton(XBOX_START_BUTTON) && !m_auxController.GetRawButton(XBOX_BACK_BUTTON); }};
//     frc2::Button m_backAux{[&] { return m_auxController.GetRawButton(XBOX_BACK_BUTTON); }};//

//     frc2::Button m_aButtonLeftBumpAux{[&] { return m_auxController.GetRawButton(XBOX_A_BUTTON) && m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//
//     frc2::Button m_aButtonAux{[&] { return m_auxController.GetRawButton(XBOX_A_BUTTON) && !m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//
//     frc2::Button m_bButtonLeftBumpAux{[&] { return m_auxController.GetRawButton(XBOX_B_BUTTON) && m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//
//     frc2::Button m_bButtonAux{[&] { return m_auxController.GetRawButton(XBOX_B_BUTTON) && !m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//

//     frc2::Button m_yButtonAux{[&] { return m_auxController.GetRawButton(XBOX_Y_BUTTON); }};//
//     frc2::Button m_xButtonAux{[&] { return m_auxController.GetRawButton(XBOX_X_BUTTON); }};//
    
//     frc2::Button m_rBumperAux{[&] { return m_auxController.GetRawButton(XBOX_RIGHT_BUMPER); }};//
//     frc2::Button m_lBumperAux{[&] { return m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//
//     frc2::Button m_rTriggerAux{[&] { return m_auxController.GetAxis(XBOX_RIGHT_TRIGGER, .5); }};//
//     frc2::Button m_lTriggerAux{[&] { return m_auxController.GetAxis(XBOX_LEFT_TRIGGER, .5); }};//
    
//     frc2::POVButton m_tDpadAux;
//     frc2::POVButton m_bDpadAux;
//     frc2::POVButton m_lDpadAux;
//     frc2::POVButton m_rDpadAux;
//     // TurretSubsystem m_turret;
    

//     void ConfigureButtonBindings();

// };
