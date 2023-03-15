// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
#include <frc2/command/InstantCommand.h>
#include "cameraserver/CameraServer.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/DataLogManager.h>
#include <frc2/command/StartEndCommand.h>
#include "frc/DriverStation.h"
#include "frc2/command/SequentialCommandGroup.h"

//auto
#include "commands/auto/CenterLaneBlueAutoCommand.h"
#include "commands/auto/CenterLaneRedAutoCommand.h"
#include "commands/auto/LeftLaneBlueAutoCommand.h"
#include "commands/auto/LeftLaneRedAutoCommand.h"
#include "commands/auto/RightLaneBlueAutoCommand.h"
#include "commands/auto/RightLaneRedAutoCommand.h"
#include "commands/auto/TestCommand.h"
#include "commands/auto/LeftLaneBlueBalanceAutoCommand.h"

//drive
#include "commands/drive/DriveWithJoystickCommand.h"

// Aquire Game Piece
#include "commands/AcquireGamePieceCommand.h"

// Score Game Piece
#include "commands/ScoreGamePieceCommand.h"

//Elevator
#include "commands/ElevatorGoToPositionCommand.h"


class InstantDisabledCommand : public frc2::InstantCommand {
public:

  InstantDisabledCommand(std::function<void()> toRun,
                 std::initializer_list<frc2::Subsystem*> requirements = {}) : frc2::InstantCommand(toRun, requirements) {} 

  virtual bool RunsWhenDisabled() const override {
    return true;
  }
};

RobotContainer::RobotContainer():m_driverController(0), m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM),
                                  m_lDpadAux(&m_auxController, XBOX_DPAD_LEFT),
                                  m_rDpadAux(&m_auxController, XBOX_DPAD_RIGHT)

{
    
    ConfigureButtonBindings();
        m_chooser.SetDefaultOption("Center Lane Blue", new CenterLaneBlueAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Center Lane Red", new CenterLaneRedAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Left Lane Blue", new LeftLaneBlueAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Left Lane Blue Balance", new LeftLaneBlueBalanceAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Left Lane Red", new LeftLaneRedAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Right Lane Blue", new RightLaneBlueAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Right Lane Red", new RightLaneRedAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        // m_chooser.AddOption("Test Auto", new TestCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        frc::SmartDashboard::PutData(&m_chooser);  
        frc::SmartDashboard::PutData("Zero Steer", new InstantDisabledCommand([this]{m_drivetrain.ResetEncoders();}));
        frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this]{m_drivetrain.ResetOdometry(frc::Pose2d());}));
        frc::SmartDashboard::PutData("Reset Angle", new InstantDisabledCommand([this]{m_drivetrain.ZeroHeading();}));
        frc::SmartDashboard::PutData("Test Auto", new TestCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        frc::SmartDashboard::PutData("Engage Elevator Brake", new InstantDisabledCommand([this] {m_elevator.EngageBrake();}));
        frc::SmartDashboard::PutData("Release Elevator Brake", new InstantDisabledCommand([this]{m_elevator.ReleaseBrake();}));
        frc::SmartDashboard::PutData("Zero Elevator", new InstantDisabledCommand([this]{m_elevator.Zero();}));
        // Cone Positions
        frc::SmartDashboard::PutData("Elevator Pos 255000", new ElevatorGoToPositionCommand(&m_elevator, 255000));
        frc::SmartDashboard::PutData("Elevator Pos 145000", new ElevatorGoToPositionCommand(&m_elevator, 145000));
        frc::SmartDashboard::PutData("Elevator Pos 80000", new ElevatorGoToPositionCommand(&m_elevator, 80000));
        
        
        frc::SmartDashboard::PutData("Elevator Pos 0", new ElevatorGoToPositionCommand(&m_elevator, 0));
        frc::SmartDashboard::PutData("DriveTrain", &m_drivetrain);
        frc::SmartDashboard::PutData("Intake", &m_intake);
        frc::SmartDashboard::PutData("Elevator", &m_elevator);

        // frc::SmartDashboard::PutData("Compressor", &m_compressor);
}


void RobotContainer::ConfigureButtonBindings() {
    // Driver Buttons
    // Driver Driving
    m_drivetrain.SetDefaultCommand(std::move(DriveWithJoystickCommand(&m_drivetrain, &m_driverController)));
    m_lBumperDriver.OnTrue(new frc2::InstantCommand([this]{m_drivetrain.toggleFieldCentricForJoystick();},{&m_drivetrain}));
    
    // Driver Acquire Game Piece TODO finish

    m_startDriver.OnTrue(new InstantDisabledCommand([this]{m_drivetrain.ZeroHeading();}));

  //intake
    m_lTriggerDriver.OnTrue(new AcquireGamePieceCommand(&m_gripper, &m_intake, &m_flipper, false, true));
    m_rTriggerDriver.OnTrue(new AcquireGamePieceCommand(&m_gripper, &m_intake, &m_flipper, false, false));

  // Operator Buttons
    // Operator Low Score Game Piece Command
    // m_aButtonAux.OnTrue(new ScoreGamePieceCommand(FLOOR, &m_elevator, &m_gripper, &m_slide));
    // // Operator Mid Score Game Piece Command
    // m_xButtonAux.OnTrue(new ScoreGamePieceCommand(MID, &m_elevator, &m_gripper, &m_slide));
    // // Operator High Score Game Piece Command
    // m_yButtonAux.OnTrue(new ScoreGamePieceCommand(TOP, &m_elevator, &m_gripper, &m_slide));
    
  //flipper
    m_lBumperAux.OnTrue(frc2::InstantCommand(
      [this] {m_flipper.Down();},{&m_flipper}
    ).ToPtr());

    m_lTriggerAux.OnTrue(frc2::InstantCommand(
      [this] {m_flipper.Up();},{&m_flipper}
    ).ToPtr());



  //elevator
    m_aButtonAux.OnTrue(
      frc2::SequentialCommandGroup{
        m_gripper.CloseCommand(),
        ElevatorGoToPositionCommand(&m_elevator, ElevatorConstants::k_ElevatorStowPosition)      
    }.WithTimeout(3.0_s));
    m_xButtonAux.OnTrue(frc2::SequentialCommandGroup{
      ElevatorGoToPositionCommand(&m_elevator, ElevatorConstants::k_ElevatorFloorPosition),
      m_flipper.DownCommand()
    }.WithTimeout(3.0_s));
    m_yButtonAux.OnTrue(frc2::SequentialCommandGroup{
      ElevatorGoToPositionCommand(&m_elevator, ElevatorConstants::k_ElevatorTopPosition),
      m_flipper.DownCommand()
    }.WithTimeout(3.0_s));
    m_bButtonAux.OnTrue(frc2::SequentialCommandGroup{
      ElevatorGoToPositionCommand(&m_elevator, ElevatorConstants::k_ElevatorMidPosition),
      m_flipper.DownCommand()
    }.WithTimeout(3.0_s));


  //intake
    // m_aButtonAux.OnTrue(new frc2::InstantCommand([this] {m_intake.Extend();},{&m_intake}));
    // m_bButtonAux.OnTrue(new frc2::InstantCommand([this] {m_intake.Retract();},{&m_intake}));
    // m_xButtonAux.OnTrue(new frc2::InstantCommand([this] {m_intake.TurnOnIntake();},{&m_intake}));
    // m_yButtonAux.OnTrue(new frc2::InstantCommand([this] {m_intake.TurnOff();},{&m_intake}));
   

    m_tDpadAux.OnTrue(frc2::InstantCommand(
      [this]{
        m_intake.TurnOff();
        m_intake.Retract();
      },{&m_intake}
      ).ToPtr());

    m_bDpadAux.OnTrue(m_intake.ExtendCommand().ToPtr());

    m_lDpadAux.WhileTrue(frc2::StartEndCommand(
      [this] {m_intake.TurnOnBarf();},
      [this] {m_intake.TurnOff();},{&m_intake}
      ).ToPtr());

  //gripper
    m_rBumperAux.OnTrue(frc2::InstantCommand(
      [this] {m_gripper.Open();}
    ).ToPtr());

    m_rTriggerAux.OnTrue(frc2::InstantCommand(
      [this] {
        if (m_flipper.IsUp()) {
          m_gripper.Close();
          m_intake.TurnOff();
          m_intake.Retract();
        }
      }, {&m_intake, &m_flipper, &m_gripper}).ToPtr());



}

frc2::Command* RobotContainer::GetAutonomousCommand(){
  return m_chooser.GetSelected();
}