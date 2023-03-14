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
      int m_endCount;
      int m_startCount;

 public:
  ElevatorGoToPositionCommand(Elevator * elevator, double pos, bool fork = false){
    m_pElevator = elevator;
    m_pos = pos;
    if(fork == false){
      AddRequirements(m_pElevator);
    }
  }

  void Initialize(){
    m_pElevator->ReleaseBrake();
    m_startCount = 0;
    // m_pElevator->SetTargetPosition(m_pos);
    m_endCount = 0;
  }

  void Execute() {

    if (++m_startCount >= 12) {

      if (m_startCount == 12) {
        m_pElevator->SetTargetPosition(m_pos);
      }
    
      if (m_pos == 0) {
        m_pElevator->SetPerceptOutput(-0.8);
        if(m_pElevator->IsInAllTheWay()){
          m_pElevator->EngageBrake();
          m_endCount++;
        }
        else if(m_endCount > 0){
          m_endCount++;
        }
      }
    }

  }

  void End(bool interrupted){
    m_pElevator->Stop();
    m_pElevator->EngageBrake();
  }

  bool IsFinished(){
    
    // Wait until the brake is released and the setpoint has changed before we check the actual is finished conditions.
    if (m_startCount <= 12) {
      return false;
    }
    
    if(m_pos == 0){
       return m_endCount > 25;
    }
    else{
       return m_pElevator->IsOnTarget() || (m_pElevator->IsInAllTheWay() && (m_pElevator->GetActualPosition() ));
     }
    // return(abs(m_pElevator->GetActualPosition() - m_pElevator->GetTargetPosition()) < ElevatorConstants::k_ElevatorOnTargetThreshold);
  }
};

