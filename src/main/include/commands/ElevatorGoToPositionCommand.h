// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ElevatorGoToPositionCommand
    : public frc2::CommandHelper<frc2::CommandBase, ElevatorGoToPositionCommand> {

      private:
      Elevator * m_pElevator;
      double m_pos;

 public:
  ElevatorGoToPositionCommand(Elevator * elevator, double pos){
    m_pElevator = elevator;
    m_pos = pos;

    AddRequirements(m_pElevator);
  }

  void Initialize(){
    m_pElevator->ReleaseBrake();
    m_pElevator->SetTargetPosition(m_pos);
  }

  void Execute() {
    if (m_pos == 0 && m_pElevator->GetActualPosition() < 100000) {
      m_pElevator->SetPerceptOutput(-0.4);
    }
  }

  void End(bool interrupted){
    m_pElevator->Stop();
    m_pElevator->EngageBrake();
  }

  bool IsFinished(){
    if(m_pos == 0){
       return m_pElevator->IsInAllTheWay();
    }
    else{
       return m_pElevator->IsOnTarget() || (m_pElevator->IsInAllTheWay() && (m_pElevator->GetActualPosition() > m_pElevator->GetTargetPosition()));
     }
    // return(abs(m_pElevator->GetActualPosition() - m_pElevator->GetTargetPosition()) < ElevatorConstants::k_ElevatorOnTargetThreshold);
  }
};
