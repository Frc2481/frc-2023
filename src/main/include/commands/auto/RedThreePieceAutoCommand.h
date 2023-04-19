// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Flipper.h"
#include "subsystems/Gripper.h"
#include "subsystems/Intake.h"
#include "subsystems/Slide.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/InstantCommand.h>
#include "RobotParameters.h"
#include "commands/ScoreGamePieceCommand.h"
#include "commands/FollowPathCommand3Piece.h"
#include <frc2/command/ParallelDeadlineGroup.h>
#include "commands/AcquireGamePieceCommand.h"
#include "commands/ScoreGamePieceCommand.h"
#include "commands/ElevatorGoToPositionCommand.h"
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>

#include <frc2/command/ProxyCommand.h>



/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RedThreePieceAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
    RedThreePieceAutoCommand> {

      private:
      Drivetrain* m_pDrive;
      Elevator* m_pElevator;
      Flipper* m_pFlipper;
      Gripper* m_pGripper;
      Intake* m_pIntake;
      Slide* m_pSlide;
      frc::Pose2d m_initialPosition;


 public:
  RedThreePieceAutoCommand(Drivetrain* drive, Elevator* elevator, Flipper* flipper, Gripper* gripper, Intake* intake, Slide* slide){
    m_pDrive = drive;
    m_pElevator = elevator;
    m_pFlipper = flipper;
    m_pGripper = gripper;
    m_pIntake = intake;
    m_pSlide = slide;

    // Need to adjust this to the start position... based off of limelight OR move the path after its created.
    
    // 0,0 is center of field.
    // +Y is left from drive station driving perspective.
    // -X is closer to driver station from center of field.
    
    // Path offset 
    // Y 141
    // X 255

    // m_initialPosition = frc::Pose2d{-255_in, 141_in, 0_deg};
    m_initialPosition = frc::Pose2d{0_in, 0_in, 0_deg};


    frc::Translation2d path_offset = m_initialPosition.Translation();

//beginning of path that turns
    frc::TrajectoryConfig forwardStartConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(18)};
    forwardStartConfig.SetKinematics(m_pDrive->GetKinematics());
    forwardStartConfig.SetReversed(false);
    forwardStartConfig.SetEndVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed));
    forwardStartConfig.SetStartVelocity(units::velocity::feet_per_second_t(5));


//end of path that turns
    frc::TrajectoryConfig forwardEndConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(18)};
    forwardEndConfig.SetKinematics(m_pDrive->GetKinematics());
    forwardEndConfig.SetReversed(false);
    forwardEndConfig.SetStartVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed));

//full path
    frc::TrajectoryConfig forwardConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(18)};
    forwardConfig.SetKinematics(m_pDrive->GetKinematics());
    forwardConfig.SetReversed(false);
    forwardConfig.SetStartVelocity(units::velocity::feet_per_second_t(5));

    // frc::TrajectoryConfig forwardMidConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed / 4),
    //                              units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel)};
    // forwardMidConfig.SetKinematics(m_pDrive->GetKinematics());
    // forwardMidConfig.SetReversed(false);
    // forwardMidConfig.SetEndVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed / 1.5));
    // forwardMidConfig.SetStartVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed / 1.5));


//beginning of path that turns
    frc::TrajectoryConfig reverseStartConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(16)};
    reverseStartConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseStartConfig.SetReversed(true);
    reverseStartConfig.SetEndVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed));
    reverseStartConfig.SetStartVelocity(units::velocity::feet_per_second_t(5));

//end of path that turns
    frc::TrajectoryConfig reverseEndConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(16)};
    reverseEndConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseEndConfig.SetReversed(true);
    reverseEndConfig.SetStartVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed));

//full path
    frc::TrajectoryConfig reverseConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed),
                                 units::acceleration::feet_per_second_squared_t(16)};
    reverseConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseConfig.SetReversed(true);
    reverseConfig.SetStartVelocity(units::velocity::feet_per_second_t(5));

    frc::TrajectoryConfig reverseFinalConfig{units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed / 3),
                                 units::acceleration::feet_per_second_squared_t(RobotParameters::k_maxAccel)};
    reverseFinalConfig.SetKinematics(m_pDrive->GetKinematics());
    reverseFinalConfig.SetReversed(true);
    reverseFinalConfig.SetEndVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed / 3));
    reverseFinalConfig.SetStartVelocity(units::velocity::feet_per_second_t(RobotParameters::k_maxSpeed));



    AddCommands(
      
     frc2::SequentialCommandGroup{
        
        frc2::InstantCommand([this]{m_pDrive->ResetOdometry(m_initialPosition);},{m_pDrive}),
        m_pGripper->CloseCommand(),
        // ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorStowPosition),

      //score first piece
        frc2::ScheduleCommand(new ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorTopPosition, true)), // Elevator goes out
        frc2::WaitCommand(0.1_s), // Wait for the the target to get set. 
        m_pFlipper->DownCommand(),
        m_pElevator->WaitForElevatorPastPositionCommand(),
        m_pGripper->OpenCommand(),
        m_pGripper->DroppedGamePieceCommand(),
        frc2::WaitCommand(0.5_s),


      //drive out first time
        frc2::ParallelDeadlineGroup{  
          // FollowPathCommand3Piece( //drive out fast
          //   frc::Pose2d{0_in, 0_in, 0_deg},
          //   {},
          //   frc::Pose2d{64_in, 0_in, 0_deg},
          //   forwardStartConfig, m_pDrive, path_offset),

          // FollowPathCommand( //drive over bump slow
          //   frc::Pose2d{64_in, 0_in, 0_deg},
          //   {},
          //   frc::Pose2d{80_in, 0_in, 0_deg},
          //   forwardMidConfig, m_pDrive, path_offset),
          
           FollowPathCommand3Piece( //keep driving out fast
            frc::Pose2d{0_in, 0_in, 0_deg},
            {frc::Translation2d{135_in, -6_in}, frc::Translation2d{160_in, -10_in}},
            frc::Pose2d{203_in, -12_in, 0_deg},
            forwardConfig, m_pDrive, path_offset),

      //pick up second piece
          frc2::SequentialCommandGroup{   
            m_pGripper->CloseCommand(), 
            ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorStowPosition),
            frc2::ScheduleCommand(new AcquireGamePieceCommand(m_pGripper, m_pIntake, m_pFlipper, true, true)), // geting 1st game piece
          }
        },


      //drive back in first time
        frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
            m_pGripper->WaitForGamePieceCommand(),
            frc2::WaitCommand(0.25_s),
            frc2::ConditionalCommand(frc2::SequentialCommandGroup{
              frc2::WaitCommand(0.25_s), // TODO: Add a comment to justify the existance of this wait.
              (ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorMidPosition, true)), // Elevator goes out
            }, 
            frc2::InstantCommand([]{}),
            [this] {return m_pGripper->GetGamePieceType() != NONE;}),
        }
        ),
          // FollowPathCommand3Piece( //driving back in 
          //   frc::Pose2d{227_in, 16_in, 0_deg},
          //   {},
          //   frc::Pose2d{114_in, 16_in, 0_deg},
          //   reverseStartConfig, m_pDrive, path_offset),

          //  FollowPathCommand3Piece( //slowing down over bump
          //   frc::Pose2d{114_in, 16_in, 0_deg},
          //   {},
          //   frc::Pose2d{84_in, 16_in, 0_deg},
          //   reverseMidConfig, m_pDrive, path_offset),

           FollowPathCommand3Piece( //keep driving in fast
            frc::Pose2d{203_in, -12_in, 0_deg},
            {frc::Translation2d{100_in, -14_in}, frc::Translation2d{50_in, -20_in}},
            frc::Pose2d{-1_in, -20_in, 0_deg}, //40
            reverseConfig, m_pDrive, path_offset),

      //score second game piece
        frc2::ConditionalCommand(frc2::SequentialCommandGroup{
          m_pElevator->WaitForElevatorPastMidPositionCommand(),
          frc2::WaitCommand(0.4_s),
          m_pFlipper->DownCommand(),
          m_pGripper->OpenCommand(),
          m_pGripper->DroppedGamePieceCommand(),
          // m_pFlipper->LaunchCommand(),
          frc2::WaitCommand(0.3_s), //0.5
          m_pFlipper->DownCommand(),
        }, 
        frc2::InstantCommand([]{}),
        [this] {return m_pGripper->GetGamePieceType() != NONE;}),

      //drive back out second time
        frc2::ParallelDeadlineGroup{
        //   FollowPathCommand3Piece( //drive out fast
        //     frc::Pose2d{4_in, 27_in, 0_deg},
        //     {frc::Translation2d{50_in, 22_in}, frc::Translation2d{60_in, 18_in}},
        //     frc::Pose2d{65_in, 16_in, 0_deg},
        //     forwardStartConfig, m_pDrive, path_offset),

        //   FollowPathCommand3Piece( //drive over bump slow
        //     frc::Pose2d{65_in, 16_in, 0_deg},
        //     {},
        //     frc::Pose2d{80_in, 16_in, 0_deg},
        //     forwardMidConfig, m_pDrive, path_offset),
          
        frc2::SequentialCommandGroup{
          FollowPathCommand3Piece( //keep driving out fast
            frc::Pose2d{-1_in, -24_in, 0_deg},
            {frc::Translation2d{40_in, -16_in}, frc::Translation2d{120_in, -20_in}},
            frc::Pose2d{140_in, -25_in, 0_deg},
            forwardStartConfig, m_pDrive, path_offset),


           FollowPathCommand3Piece( //keep driving out fast and turn
            frc::Pose2d{140_in, -25_in, 0_deg},
            {},
            frc::Pose2d{203_in, -64_in, -30_deg}, //196
            forwardEndConfig, m_pDrive, path_offset),
        },
      //pick up third game piece
          frc2::SequentialCommandGroup{
            m_pGripper->CloseCommand(),
            ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorStowPosition),
            frc2::ScheduleCommand(new AcquireGamePieceCommand(m_pGripper, m_pIntake, m_pFlipper, true, true)), // geting 2nd game piece
          }
        },

      //drive back in second time
          frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
            m_pGripper->WaitForGamePieceCommand(),
            frc2::WaitCommand(1.0_s),
            frc2::ConditionalCommand(frc2::SequentialCommandGroup{
              frc2::WaitCommand(0.25_s),
              (ElevatorGoToPositionCommand(m_pElevator, ElevatorConstants::k_ElevatorTopPosition, true)), // Elevator goes out
              m_pFlipper->DownCommand(),
            }, 
            frc2::InstantCommand([]{}),
            [this] {return m_pGripper->GetGamePieceType() != NONE;}),
        }
        ),
          // FollowPathCommand3Piece( //driving back in 
          //   frc::Pose2d{200_in, 68_in, 0_deg},
          //   {frc::Translation2d{160_in, 45_in}, frc::Translation2d{145_in, 25_in}},
          //   frc::Pose2d{114_in, 16_in, 0_deg},
          //   reverseStartConfig, m_pDrive, path_offset),

          //  FollowPathCommand3Piece( //slowing down over bump
          //   frc::Pose2d{114_in, 16_in, 0_deg},
          //   {},
          //   frc::Pose2d{84_in, 16_in, 0_deg},
          //   reverseMidConfig, m_pDrive, path_offset),

           FollowPathCommand3Piece( //keep driving in fast and turn
            frc::Pose2d{203_in, -64_in, -30_deg},
            {frc::Translation2d{180_in, -50_in}, frc::Translation2d{175_in, -40_in}},
            frc::Pose2d{150_in, -20_in, 0_deg},
            reverseStartConfig, m_pDrive, path_offset),

          FollowPathCommand3Piece( //keep driving in fast
            frc::Pose2d{150_in, -20_in, 0_deg},
            {frc::Translation2d{100_in, -24_in}, frc::Translation2d{60_in, -36_in}},
            frc::Pose2d{5_in, -38_in, 0_deg}, //-4, 48
            reverseEndConfig, m_pDrive, path_offset),

          FollowPathCommand3Piece( //keep driving in fast
            frc::Pose2d{5_in, -38_in, 0_deg},
            {},
            frc::Pose2d{-12_in, -44_in, 0_deg}, //-4, 48
            reverseFinalConfig, m_pDrive, path_offset),
        
      //score third game piece
         frc2::ConditionalCommand(frc2::SequentialCommandGroup{
          m_pElevator->WaitForElevatorPastPositionCommand(),
          frc2::WaitCommand(0.25_s),
          m_pGripper->OpenCommand(),
          frc2::WaitCommand(0.5_s),
          m_pGripper->DroppedGamePieceCommand(),
            }, 
            frc2::InstantCommand([]{}),
            [this] {return m_pGripper->GetGamePieceType() != NONE;}),
  });  
      }
  };

