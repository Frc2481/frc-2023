// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "components/Joystick2481.h"
#include "subsystems/Drivetrain.h"
#include "components/XboxController2481.h"
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "RobotContainer.h"
#include <frc/filter/SlewRateLimiter.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveWithJoystickCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveWithJoystickCommand> {
 public:
  DriveWithJoystickCommand(Drivetrain* swerveDrivetrain, Joystick2481* driverController){
    m_pDrivetrain = swerveDrivetrain;
    m_pDriverController = driverController;
    AddRequirements(m_pDrivetrain);
  }

  void Initialize() override{
    
  }

  void Execute() override{
    m_fieldCentric = m_pDrivetrain->getFieldCentricForJoystick();
    //if (!m_rBumperDriver.GetRawButton(XBOX_RIGHT_BUMPER)) {
            if(m_fieldCentric){
              m_pDrivetrain->Drive(units::meters_per_second_t(m_pDriverController->GetRawAxis(XBOX_LEFT_Y_AXIS)),
                                units::meters_per_second_t(-m_pDriverController->GetRawAxis(XBOX_LEFT_X_AXIS)),
                                units::radians_per_second_t(m_pDriverController->GetRawAxis(XBOX_RIGHT_X_AXIS)*2)//*6.65
                                );
            }else{
              m_pDrivetrain->Drive(units::meters_per_second_t(m_pDriverController->GetRawAxis(XBOX_LEFT_Y_AXIS)),
                                units::meters_per_second_t(m_pDriverController->GetRawAxis(XBOX_LEFT_X_AXIS)),
                                units::radians_per_second_t(m_pDriverController->GetRawAxis(XBOX_RIGHT_X_AXIS)*2)//*6.65
                                );
            }
    
    //}
  }

  void End(bool interrupted) override{
    m_pDrivetrain->Drive(0_mps, 0_mps, units::radians_per_second_t(0));
  }

  bool IsFinished() override{
    return false;
  }
  void ToggleFeildCentric(){
    if(m_fieldCentric){
      m_fieldCentric = false;
    }else{
      m_fieldCentric = true;
    }
  }

 private:
    Joystick2481* m_pDriverController;
    Drivetrain* m_pDrivetrain;
    bool m_fieldCentric = false;
    
};
