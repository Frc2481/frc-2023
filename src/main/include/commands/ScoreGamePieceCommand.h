// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/Elevator.h"
#include "subsystems/Gripper.h"
#include "subsystems/Slide.h"
#include <frc2/command/SelectCommand.h>



enum ScoringPosition{FLOOR, MID_SHELF, TOP_SHELF, BOTTOM_POST, TOP_POST};
enum GamePieceType{CUBE, CONE};

class ScoreGamePieceCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
      ScoreGamePieceCommand> {

      private:
      Elevator* m_pElevator;
      Gripper* m_pGripper;
      Slide* m_pSlide;
      ScoringPosition m_scoringPosition;
      GamePieceType m_gamePieceType;

 public:
  ScoreGamePieceCommand(ScoringPosition scoringPosition, GamePieceType gamePieceType, Elevator* elevator, Gripper* gripper, Slide* slide){
    m_pElevator = elevator;
    m_pGripper = gripper;
    m_pSlide = slide;
    m_scoringPosition = scoringPosition;
    m_gamePieceType = gamePieceType;

  AddCommands(
      frc2::SequentialCommandGroup{
        frc2::SelectCommand<ScoringPosition>{[this] {return m_scoringPosition;}, 
          std::pair{FLOOR, m_pElevator->GoToFloorCommand()},
          std::pair{MID_SHELF, m_pElevator->GoToMidShelfCommand()},
          std::pair{TOP_SHELF, m_pElevator->GoToTopShelfCommand()},
          std::pair{BOTTOM_POST, m_pElevator->GoToBottomPostCommand()},
          std::pair{TOP_POST, m_pElevator->GoToTopPostCommand()},
        },
        frc2::SelectCommand<GamePieceType>{[this] {return m_gamePieceType;},
          std::pair{CUBE, frc2::SelectCommand<ScoringPosition>{[this] {return m_scoringPosition;}, 
                            std::pair{FLOOR, m_pSlide->TrackAprilTagsMidShelfCommand()},
                            std::pair{MID_SHELF, m_pSlide->TrackAprilTagsMidShelfCommand()},
                            std::pair{TOP_SHELF, m_pSlide->TrackAprilTagsTopShelfCommand()},
                          }
          },
          std::pair{CONE, frc2::SelectCommand<ScoringPosition>{[this] {return m_scoringPosition;}, 
                            std::pair{FLOOR, m_pSlide->TrackLimelightBottomPostCommand()},
                            std::pair{BOTTOM_POST, m_pSlide->TrackLimelightBottomPostCommand()},
                            std::pair{TOP_POST, m_pSlide->TrackLimelightTopPostCommand()},
                          }
          }
        },
        m_pSlide->WaitForSlideOnTargetCommand(),
        m_pElevator->WaitForElevatorOnTargetCommand()
        //TODO drop and center hatch slide and retract elevator

      
      }
    );
  }
};
