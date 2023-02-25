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
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>


enum ScoringPosition{FLOOR, MID, TOP};


class ScoreGamePieceCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
      ScoreGamePieceCommand> {

private:
    Elevator* m_pElevator;
    Gripper* m_pGripper;
    Slide* m_pSlide;
    ScoringPosition m_scoringPosition;
   

public:
  ScoreGamePieceCommand(ScoringPosition scoringPosition, Elevator* elevator, Gripper* gripper, Slide* slide){
    m_pElevator = elevator;
    m_pGripper = gripper;
    m_pSlide = slide;
    m_scoringPosition = scoringPosition;
    

    AddCommands(
      frc2::SequentialCommandGroup{
        frc2::ScheduleCommand(
          new frc2::SelectCommand<GamePieceType>{[this] {return m_pGripper->GetGamePieceType();},
            std::pair{CUBE, frc2::SelectCommand<ScoringPosition>{[this] {return m_scoringPosition;}, 
                              std::pair{FLOOR, frc2::SequentialCommandGroup{
                                    m_pElevator->GoToFloorCommand(),
                                    m_pSlide->TrackAprilTagsMidShelfCommand(),
                                  },
                              },
                              std::pair{MID, frc2::SequentialCommandGroup{
                                    m_pElevator->GoToMidShelfCommand(),
                                    m_pSlide->TrackAprilTagsMidShelfCommand(),
                                },
                              },
                              std::pair{TOP, frc2::SequentialCommandGroup{
                                    m_pElevator->GoToTopShelfCommand(),
                                    m_pSlide->TrackAprilTagsTopShelfCommand(),
                                },
                              },
                            },
              },
            std::pair{CONE, frc2::SelectCommand<ScoringPosition>{[this] {return m_scoringPosition;}, 
                              std::pair{FLOOR, frc2::SequentialCommandGroup{
                                    m_pElevator->GoToFloorCommand(),
                                    m_pSlide->TrackLimelightBottomPostCommand(),
                                },
                              },
                              std::pair{MID, frc2::SequentialCommandGroup{
                                    m_pElevator->GoToBottomPostCommand(),
                                    m_pSlide->TrackLimelightBottomPostCommand(),
                                },
                              },
                              std::pair{TOP, frc2::SequentialCommandGroup{
                                    m_pElevator->GoToTopPostCommand(),
                                    m_pSlide->TrackLimelightTopPostCommand(),
                                },
                              },
                            },
            },
            std::pair{NONE, frc2::PrintCommand("Unknown Game Piece, Cannot Score")}
          }
        ),
        m_pSlide->WaitForTargetVisibleCommand(),
        m_pSlide->WaitForSlideOnTargetCommand(),
        m_pElevator->WaitForElevatorOnTargetCommand(),
        m_pGripper->OpenCommand(),
        frc2::WaitCommand(0.5_s),  // Time for cone/cube to fall
        m_pSlide->GoToCenterPositionCommand(),
        m_pElevator->StowCommand()
      }
    );
  }
};
