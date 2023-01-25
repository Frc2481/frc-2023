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
#include "commands/auto/LeftLaneBlueCommand.h"
#include "commands/auto/LeftLaneRedCommand.h"
#include "commands/auto/RightLaneBlueCommand.h"
#include "commands/auto/RightLaneRedCommand.h"



// RobotContainer::RobotContainer():m_driverController(0), m_auxController(1),
//                                   m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
//                                   m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM),
//                                   m_lDpadAux(&m_auxController, XBOX_DPAD_LEFT),
//                                   m_rDpadAux(&m_auxController, XBOX_DPAD_RIGHT)

// {
// ConfigureButtonBindings();
//   m_chooser.SetDefaultOption("Center Lane Blue", new CenterLaneBlueAutoCommand());
//   m_chooser.AddOption("Center Lane Red", new CenterLaneRedAutoCommand());
//   m_chooser.AddOption("Left Lane Blue", new LeftLaneBlueCommand());
//   m_chooser.AddOption("Left Lane Red", new LeftLaneRedCommand());
//   m_chooser.AddOption("Right Lane Blue", new RightLaneBlueCommand());
//   m_chooser.AddOption("Right Lane Red", new RightLaneRedCommand());
//   frc::SmartDashboard::PutData(&m_chooser);  
// // }
