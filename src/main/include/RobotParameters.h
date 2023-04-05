
#include <math.h>
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
// #include <units/units.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <numbers>

#define COMP

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace PDPChannels{
    static constexpr int kIntake = 10; //find
}

namespace TalonIDs{
    
}

namespace TalonSRXIDs{
    
    // static constexpr int kIndexerRollerMotorID = 12;
    static constexpr int kFrontRightTurningMotorID = 6;
    static constexpr int kFrontLeftTurningMotorID = 3;
    static constexpr int kBackRightTurningMotorID = 12;
    static constexpr int kBackLeftTurningMotorID = 9;
}

namespace FalconIDs{
    static constexpr int kFrontRightDriveMotorID = 4;
    static constexpr int kFrontRightDriveMotorFollowerID = 5;
    static constexpr int kFrontLeftDriveMotorID = 1;
    static constexpr int kFrontLeftDriveMotorFollowerID = 2;
    static constexpr int kBackRightDriveMotorID = 10;
    static constexpr int kBackRightDriveMotorFollowerID = 11;
    static constexpr int kBackLeftDriveMotorID = 7;
    static constexpr int kBackLeftDriveMotorFollowerID = 8;
    static constexpr int kElevatorMotor = 13;
    static constexpr int kSlideMotor = 14;
    static constexpr int kIntakeHorizontalMotor = 20;
    static constexpr int kIntakeHorizontalMotorFollower = 15;
    static constexpr int kIntakeVerticalMotor = 21;
    static constexpr int kFlipperMotorID = 16;
} 

namespace CANCoderIDs
{ // bot used
    // static constexpr int kFrontRightSteerCANCoderID = 3;
    // static constexpr int kFrontLeftSteerCANCoderID = 1;
    // static constexpr int kRearRightSteerCANCoderID = 4;
    // static constexpr int kRearLeftSteerCANCoderID = 2;
    // static constexpr int kRearMiddleSteerCANCoderID = 5; // no middle
}

namespace SolenoidPorts{
 
    // Old PH Ports
    // static constexpr int kIntakeFirstSolenoidPort = 12;
    // static constexpr int kIntakeFirstSolenoidPortIn = 3;
    // static constexpr int kIntakeSecondSolenoidPort = 11;
    // static constexpr int kGripperSolenoidPort = 0;
    // static constexpr int kGripperSolenoidReversePort = 15;
    // static constexpr int kFlipperSolenoidPort = 13;
    // static constexpr int kFlipperSolenoidFloatPort = 2;
    // static constexpr int kElevatorEngageBrakePort = 14;
    // static constexpr int kElevatorReleaseBrakePort = 1;

    static constexpr int kIntakeFirstSolenoidPort = 4;
    static constexpr int kIntakeFirstSolenoidPortIn = 3;
    static constexpr int kIntakeSecondSolenoidPort = 11;
    static constexpr int kGripperSolenoidPort = 0;
    static constexpr int kGripperSolenoidReversePort = 7;
    static constexpr int kFlipperSolenoidPort = 5;
    static constexpr int kFlipperSolenoidFloatPort = 2;
    static constexpr int kElevatorEngageBrakePort = 6;
    static constexpr int kElevatorReleaseBrakePort = 1;

}

// namespace DriveConstants { // not used

// constexpr bool kFrontLeftTurningEncoderReversed = false;
// constexpr bool kRearLeftTurningEncoderReversed = false;
// constexpr bool kFrontRightTurningEncoderReversed = false;
// constexpr bool kRearRightTurningEncoderReversed = false;
// constexpr bool kRearMiddleTurningEncoderReversed = false; // no middle

// static constexpr units::meters_per_second_t  kAutoDriveSpeed = 0.5_mps; //TODO fix

// constexpr bool kFrontLeftDriveEncoderReversed = true;
// constexpr bool kRearLeftDriveEncoderReversed = true;
// constexpr bool kFrontRightDriveEncoderReversed = true;
// constexpr bool kRearRightDriveEncoderReversed = true;
// constexpr bool kRearMiddleDriveEncoderReversed = true; // no middle

// constexpr bool kGyroReversed = false;

// constexpr units::meters_per_second_t kDriveClimbSpeed = 0.5_mps;

// }  // // namespace DriveConstants

// namespace ModuleConstants {
// constexpr int kEncoderCPR = 4096;
// constexpr double kWheelDiameterMeters = .15;
// constexpr double kDriveEncoderDistancePerPulse =
//     // Assumes the encoders are directly mounted on the wheel shafts
//     (kWheelDiameterMeters * std::numbers::pi) / static_cast<double>(kEncoderCPR);

// constexpr double kTurningEncoderDistancePerPulse =
//     // Assumes the encoders are directly mounted on the wheel shafts
//     (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

// constexpr double kPModuleTurningController = 1;
// constexpr double kPModuleDriveController = 1;
// }  // namespace ModuleConstants

// namespace AutoConstants { // // not used
// using radians_per_second_squared_t =
    // units::compound_unit<units::radians,
                        //  units::inverse<units::squared<units::second>>>;

// // constexpr auto kMaxSpeed = units::meters_per_second_t(3);
// constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3);
// constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
// constexpr auto kMaxAngularAcceleration =
    // units::unit_t<radians_per_second_squared_t>(3.142);

// constexpr double kPXController = 0.5;
// constexpr double kPYController = 0.5;
// constexpr double kPThetaController = 0.5;

// extern const frc::TrapezoidProfile<units::radians>::Constraints
    // kThetaControllerConstraints;

// }  // // namespace AutoConstants

// enum class CommonModes{
//     DutyCycle = 0,
//     Velocity = 1,
//     Voltage = 2,
//     Position = 3,
//     SmartMotion = 4,
//     Current = 5,
//     SmartVelocity = 6,
//     PercentOutput = 7,
//     Follower = 8,
//     MotionProfile = 9,
//     MotionMagic = 10,
//     MotionProfileArc = 11,
//     Disabled = 15
// };

namespace DigitalInputs{
    static constexpr int k_IntakeBeambreakPort = 8;
    static constexpr int k_ElevatorBeambreakPort = 9;
}

namespace IntakeConstants{
    static constexpr double k_DefaultIntakeRollerSpeed = 1; //TODO: Find out
    static constexpr double k_IntakeCurrentBallDetectThreshhold = 23; //12; //TODO find out previous 7.5
    static constexpr double k_IntakeHorizontalRollerSpeed = -1;//-0.5;
    static constexpr double k_IntakeVerticalRollerSpeed = -0.5;//-0.35;
    static constexpr double k_IntakeHorizontalRollerSpeedCube = -0.5 / 1.5;//-0.5;
    static constexpr double k_IntakeVerticalRollerSpeedCube = -0.3;//-0.35;
    
    static constexpr double k_IntakeBarfHorizontalRollerSpeed = 1;
    static constexpr double k_IntakeBarfVerticalRollerSpeed = 1;
    static constexpr double k_IntakeHorizontalCurrentLimit = 0;
    static constexpr double k_IntakeVerticalCurrentLimit = 0;
    static constexpr double k_IntakeCurrentDuration = 0;
}

namespace LimeLightConstants{
    static constexpr double k_MarkersPipeline = 0;
    static constexpr double k_TopPostPipeLine = 1;
    static constexpr double k_BottomPostPipeLine = 2;
    static constexpr double k_TopPostHeight_in = 0;
    static constexpr double k_BottomPostHeight_in = 24;
    static constexpr double k_CameraHeight_in = 14.25;
    static constexpr double k_AprilTagHeight_in = 0;
    static constexpr double k_CameraAngle = 8;
    static constexpr double k_FinalXOffset_in = 22;
}

namespace FlipperConstants{
    static constexpr double k_FlipperTopSoftLimit = 25428;
    static constexpr double k_FlipperCubeTopSoftLimit = 16502;
    static constexpr double k_FlipperBottomSoftLimit = 0;
    static constexpr double k_FlipperConeSpeed = 0.40;
    static constexpr double k_FlipperCubeSpeed = 0.20;
    static constexpr double k_FlipperCubeLaunchSpeed = 0.40;
    static constexpr double k_FlipperDownSpeed = -0.20;
}

namespace ElevatorConstants{
    static constexpr double k_ElevatorkP = 0.025;
    static constexpr double k_ElevatorkI = 0;
    static constexpr double k_ElevatorkD = 0;
    static constexpr double k_ElevatorkF = 0.047;
    static constexpr double k_ElevatorAcceleration = 120000; //36000
    static constexpr double k_ElevatorMaxSpeed = 20479 / 1.2; //24000
    static constexpr double k_ElevatorSCurveStrength = 6;
    static constexpr double k_ElevatorTicksPerInch = 0;
    static constexpr double k_ElevatorTopSoftLimit = 255000 / 2.9166 / 1.071;
    static constexpr double k_ElevatorBottomSoftLimit = 0;
    static constexpr double k_ElevatorOnTargetThreshold = 1000;
    static constexpr double k_ElevatorTopPosition = 255000 / 2.91666 / 1.071;
    static constexpr double k_ElevatorMidPosition = 135000 / 2.91666 / 1.071;
    static constexpr double k_ElevatorFloorPosition = 70000 / 2.91666 / 1.071;
    static constexpr double k_ElevatorStowPosition = 0;
}

namespace SlideConstants{
    static constexpr double k_SlidekP = 0;
    static constexpr double k_SlidekI = 0;
    static constexpr double k_SlidekD = 0;
    static constexpr double k_SlidekF = 0;
    static constexpr double k_SlideAcceleration = 0; //36000
    static constexpr double k_SlideMaxSpeed = 0; //24000
    static constexpr double k_SlideSCurveStrength = 6;
    static constexpr double k_SlideTicksPerInch = 0;
    static constexpr double k_SlideOnTargetThreshold = 0;
    static constexpr double k_SlideLeftSoftLimit = 0;
    static constexpr double k_SlideRightSoftLimit = 0;
    static constexpr double k_SlideCenterPosition = 0;
}

//TODO: figure this all out
namespace RobotParametersCompetition { 
    // robot main
	static constexpr unsigned k_updateRate = 50; // Hz

    static constexpr double k_wheelBase = 1; // in
    static constexpr double k_wheelTrack = 1; // in
    static constexpr double k_wheelLeverArm = sqrt(std::pow(k_wheelBase/2,2) + std::pow(k_wheelTrack/2,2));
    static constexpr double k_wheelRad = (3.875 / 2) *.0254; // in TODO find actual size
    static constexpr double k_wheelCirc = k_wheelRad * std::numbers::pi * 2;
    static constexpr units::feet_per_second_t k_maxSpeed = units::feet_per_second_t(13.2); //TODO change also in driveWithJoystickCommand 
    static constexpr double k_maxAccel = 8;
    static constexpr double k_maxDeccel = 1;
    static constexpr double k_steerEncoderToWheelGearRatio = 1; 
    static constexpr double k_driveMotorGearRatio = 1 / ((50 / 12.0) * (17 / 27.0) * (45 / 15.0)); //38250/6480 or 425/72 // 1 / 7.714 (11.0/30.0)*(1.0/3.0);  // 14 -> 16  6.75:1  
    static constexpr double k_ticksPerRev = 2048.0;//ticks per 100ms TODO check
    static constexpr double k_ticksPerWheelRev = k_ticksPerRev / k_driveMotorGearRatio;
    static constexpr double k_distancePerTick = k_wheelCirc / k_ticksPerWheelRev;
    static constexpr double k_driveMotorEncoderTicksToMPS = k_distancePerTick * 10 * 0.983;
    static constexpr double k_driveMotorEncoderTicksToMeters = k_distancePerTick * 0.983; //0.983 is percent error to scale
    static constexpr double k_minRobotVelocity = 1;
    static constexpr double k_minRobotYawRate = 1;
    static constexpr double k_driveWheelSlotError = 1;
    static constexpr double k_robotWidth = 1;
    static constexpr double k_robotLength = 1;
    static constexpr units::radians_per_second_t k_maxYawRate = units::radians_per_second_t(k_maxSpeed.value() / k_wheelLeverArm);
    static constexpr units::radians_per_second_squared_t k_maxYawAccel = units::radians_per_second_squared_t(k_maxAccel / k_wheelLeverArm);
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/std::numbers::pi;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180/std::numbers::pi;
    static constexpr double k_yawKp = 10;
    static constexpr double k_yawKi = 0;
    static constexpr double k_yawKd = 0;
    static constexpr double k_xyKp = 2;
    static constexpr double k_xyKi = 0;
    static constexpr double k_xyKd = 0;
    // static constexpr double k_driveMotorEncoderMPSToRPM  = (RobotParameters::k_driveMotorGearRatio/(RobotParameters::k_wheelRad*3.14159265*2))*60;

    // TODO check rest

    //pathfollowing 
    static constexpr double  k_maxCentripAccel = 10.0;//10

    // // steer motors
    static constexpr double k_steerMotorControllerKp = 3;
    static constexpr double k_steerMotorControllerKi = 0;
    static constexpr double k_steerMotorControllerKd = 40;
    static constexpr double k_steerMotorControllerKsf = 0;
    static constexpr double k_steerMotorControllerKv = 0;
    static constexpr double k_steerMotorControllerKap = 0;
    static constexpr double k_steerMotorControllerKan = 0;

    // drive motors
    static constexpr double k_driveMotorControllerKp = 0.1;
    static constexpr double k_driveMotorControllerKi = 0;
    static constexpr double k_driveMotorControllerKd = 0;
    static constexpr double k_driveMotorControllerKv = 0.061143;
    static constexpr double k_driveMotorControllerKs = 0.5913;
    static constexpr double k_driveMotorControllerKa = 0.037762;



    // encoders
    static constexpr unsigned k_ctreMagEncoderTicksPerRev = 4096;
    static constexpr unsigned k_grayhillEncoderTicksPerRev = 512;
    static constexpr unsigned k_falconFXEncoderTicksPerRev = 2048;
    
}
namespace RobotParametersTest { 
    // robot main
	static constexpr unsigned k_updateRate = 50; // Hz

    static constexpr double k_wheelBase = 1; // in
    static constexpr double k_wheelTrack = 1; // in
    static constexpr double k_wheelLeverArm = sqrt(std::pow(k_wheelBase/2,2) + std::pow(k_wheelTrack/2,2));
    static constexpr double k_wheelRad = (3.8/2)*.0254; // in TODO find actual size
    static constexpr double k_wheelCirc = k_wheelRad * std::numbers::pi * 2;
    static constexpr units::feet_per_second_t k_maxSpeed = units::feet_per_second_t(12.6); //TODO change also in driveWithJoystickCommand 
    static constexpr double k_maxAccel = 10;
    static constexpr double k_maxDeccel = 1;
    static constexpr double k_steerEncoderToWheelGearRatio = 1; 
    static constexpr double k_driveMotorGearRatio = (11.0/30.0)*(1.0/3.0);
    static constexpr double k_ticksPerRev = 2048.0;//ticks per 100ms TODO check
    static constexpr double k_ticksPerWheelRev = k_ticksPerRev / k_driveMotorGearRatio;
    static constexpr double k_distancePerTick = k_wheelCirc / k_ticksPerWheelRev;
    static constexpr double k_driveMotorEncoderTicksToMPS = k_distancePerTick * 10 * 0.983;
    static constexpr double k_driveMotorEncoderTicksToMeters = k_distancePerTick * 0.983; //0.983 is percent error to scale
    // static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;
    // static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*0.983; //0.983 is percent error to scale
    static constexpr double k_minRobotVelocity = 1;
    static constexpr double k_minRobotYawRate = 1;
    static constexpr double k_driveWheelSlotError = 1;
    static constexpr double k_robotWidth = 1;
    static constexpr double k_robotLength = 1;
    static constexpr units::radians_per_second_t k_maxYawRate = units::radians_per_second_t(k_maxSpeed.value() / k_wheelLeverArm);
    static constexpr units::radians_per_second_squared_t k_maxYawAccel = units::radians_per_second_squared_t(k_maxAccel / k_wheelLeverArm);
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/std::numbers::pi;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180/std::numbers::pi;
    static constexpr double k_yawKp = 10;
    static constexpr double k_yawKi = 0;
    static constexpr double k_yawKd = 0;
    static constexpr double k_xyKp = 2;
    static constexpr double k_xyKi = 0;
    static constexpr double k_xyKd = 0;
    // static constexpr double k_driveMotorEncoderMPSToRPM  = (RobotParameters::k_driveMotorGearRatio/(RobotParameters::k_wheelRad*3.14159265*2))*60;

    // TODO check rest

    //pathfollowing 
    static constexpr double  k_maxCentripAccel = 10.0;//10

    // // steer motors
    static constexpr double k_steerMotorControllerKp = 3;
    static constexpr double k_steerMotorControllerKi = 0;
    static constexpr double k_steerMotorControllerKd = 40;
    static constexpr double k_steerMotorControllerKsf = 0;
    static constexpr double k_steerMotorControllerKv = 0;
    static constexpr double k_steerMotorControllerKap = 0;
    static constexpr double k_steerMotorControllerKan = 0;

    // drive motors
    static constexpr double k_driveMotorControllerKp = 0.1;
    static constexpr double k_driveMotorControllerKi = 0;
    static constexpr double k_driveMotorControllerKd = 0;
    static constexpr double k_driveMotorControllerKv = 0.06666;
    static constexpr double k_driveMotorControllerKs = 0.12032;
    static constexpr double k_driveMotorControllerKa = 0.0049351;


    // encoders
    static constexpr unsigned k_ctreMagEncoderTicksPerRev = 4096;
    static constexpr unsigned k_grayhillEncoderTicksPerRev = 512;
    static constexpr unsigned k_falconFXEncoderTicksPerRev = 2048;
    
}
#ifdef COMP
#define RobotParameters RobotParametersCompetition
#else
#define RobotParameters RobotParametersTest
#endif


#endif // ROBOT_PARAMETERS_H