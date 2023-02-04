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

//auto
#include "commands/auto/CenterLaneBlueAutoCommand.h"
#include "commands/auto/CenterLaneRedAutoCommand.h"
#include "commands/auto/LeftLaneBlueAutoCommand.h"
#include "commands/auto/LeftLaneRedAutoCommand.h"
#include "commands/auto/RightLaneBlueAutoCommand.h"
#include "commands/auto/RightLaneRedAutoCommand.h"

//drive
#include "commands/drive/DriveWithJoystickCommand.h"

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
        m_chooser.AddOption("Left Lane Red", new LeftLaneRedAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Right Lane Blue", new RightLaneBlueAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        m_chooser.AddOption("Right Lane Red", new RightLaneRedAutoCommand(&m_drivetrain, &m_elevator, &m_flipper, &m_gripper, &m_intake, &m_slide));
        frc::SmartDashboard::PutData(&m_chooser);  
        frc::SmartDashboard::PutData("Zero Steer", new InstantDisabledCommand([this]{m_drivetrain.ResetEncoders();}));
        frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this]{m_drivetrain.ResetOdometry(m_drivetrain.GetOdometryPosition());}));
        frc::SmartDashboard::PutData("Reset Angle", new InstantDisabledCommand([this]{m_drivetrain.ZeroHeading();})); // check if right
}
void RobotContainer::ConfigureButtonBindings() {
    m_drivetrain.SetDefaultCommand(std::move(DriveWithJoystickCommand(&m_drivetrain, &m_driverController)));
    m_lBumperDriver.OnTrue(new frc2::InstantCommand([this]{m_drivetrain.toggleFieldCentricForJoystick();},{&m_drivetrain}));
    
}

