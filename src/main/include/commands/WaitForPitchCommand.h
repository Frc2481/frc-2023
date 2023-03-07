// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WaitForPitchCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForPitchCommand> {

      private:
      Drivetrain * m_pDrivetrain;
      float m_pitch;
      bool m_increasing;

 public:
  WaitForPitchCommand(Drivetrain * drivetrain, float pitch){
    m_pDrivetrain = drivetrain;
    m_pitch = pitch;
  }

  void Initialize(){
    m_increasing = m_pitch > m_pDrivetrain->GetPitch().Degrees().value();
  }

  bool IsFinished(){
    return (m_pDrivetrain->GetPitch().Degrees().value() > m_pitch && m_increasing) || 
           (m_pDrivetrain->GetPitch().Degrees().value() < m_pitch && !m_increasing);
     
  }
};
