// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotContainer.h"

#include "subsystems/Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousInit() override {
    m_autoCommand = m_Container.GetAutonomousCommand();
    if(m_autoCommand != nullptr){
      m_autoCommand->Schedule();
    }
  }

  void AutonomousPeriodic() override {
    // m_swerve.UpdateOdometry();
  }

   void TeleopInit() override {
    if(m_autoCommand != nullptr){
      m_autoCommand->Cancel();
      m_autoCommand = nullptr;
    }
  }

  void TeleopPeriodic(){
    
  }

  void RobotPeriodic(){
    frc2::CommandScheduler::GetInstance().Run();
  }

 private:
  frc::XboxController m_controller{0};
  // Drivetrain m_swerve;
  RobotContainer m_Container;
  frc2::Command* m_autoCommand = nullptr;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
 
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
