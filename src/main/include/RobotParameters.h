
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

namespace VictorIDs{
    
    // static constexpr int kIndexerRollerMotorID = 12;
    static constexpr int kFrontRightTurningMotorID = 33;
    static constexpr int kFrontLeftTurningMotorID = 31;
    static constexpr int kRearRightTurningMotorID = 34;
    static constexpr int kRearLeftTurningMotorID = 32;
    static constexpr int kRearMiddleTurningMotorID = 35;
    static constexpr int kFeederMotorID = 11;
    static constexpr int kIndexerMotorID = 12;
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
    static constexpr int kIntakeHorizontalMotor = 1;
    static constexpr int kIntakeVerticalMotor = 1;
} 

namespace CANCoderIDs
{
    static constexpr int kFrontRightSteerCANCoderID = 3;
    static constexpr int kFrontLeftSteerCANCoderID = 1;
    static constexpr int kRearRightSteerCANCoderID = 4;
    static constexpr int kRearLeftSteerCANCoderID = 2;
    static constexpr int kRearMiddleSteerCANCoderID = 5;
}

namespace SolenoidPorts{
 
    static constexpr int kIntakeSolenoidPort = 0;
    static constexpr int kIntakeSolenoidReversePort = 1;
    static constexpr int kGripperSolenoidPort = 2;
    static constexpr int kGripperSolenoidReversePort = 3;
    static constexpr int kFlipperSolenoidPort = 4;
    static constexpr int kFlipperSolenoidReversePort = 5;
   
    static constexpr int kFloorClimberSolenoidPort = 2;
    static constexpr int kFloorClimberSolenoidReversePort = 3;
    static constexpr int kTrussClimberSolenoidPort = 5; //We don't actuallly know if these are the right IDs
    static constexpr int kTrussClimberSolenoidReversePort = 4; //We don't actuallly know if these are the right IDs
    static constexpr int kJavelinSolenoidPort = 7;
    static constexpr int kJavelinReverseSolenoidPort = 6;
}

namespace DriveConstants {

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = false;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = false;
constexpr bool kRearMiddleTurningEncoderReversed = false;

static constexpr units::meters_per_second_t  kAutoDriveSpeed = 0.5_mps; //TODO fix

constexpr bool kFrontLeftDriveEncoderReversed = true;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = true;
constexpr bool kRearRightDriveEncoderReversed = true;
constexpr bool kRearMiddleDriveEncoderReversed = true;

constexpr bool kGyroReversed = false;

constexpr units::meters_per_second_t kDriveClimbSpeed = 0.5_mps;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr int kEncoderCPR = 4096;
constexpr double kWheelDiameterMeters = .15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * std::numbers::pi) / static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

// constexpr auto kMaxSpeed = units::meters_per_second_t(3);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants


enum class CommonModes{
    DutyCycle = 0,
    Velocity = 1,
    Voltage = 2,
    Position = 3,
    SmartMotion = 4,
    Current = 5,
    SmartVelocity = 6,
    PercentOutput = 7,
    Follower = 8,
    MotionProfile = 9,
    MotionMagic = 10,
    MotionProfileArc = 11,
    Disabled = 15
};
namespace PathConstants{ // TODO check
    static constexpr double kMinLookAhead = 6*.0254;
    static constexpr double kMaxLookAhead = 24*.0254;
}
namespace FieldConstants{
    static constexpr double kDistanceTargetToOriginY = 324;
    static constexpr double kDistanceTargetToOriginX = 162;
    // static constexpr double kRedAllianceOrigin = ;
    // static constexpr double kBlueAllianceOrigin = ;
    }
namespace IntakeConstants{
    static constexpr double k_DefaultIntakeRollerSpeed = 1; //TODO: Find out
    static constexpr double k_IntakeCurrentBallDetectThreshhold = 23; //12; //TODO find out previous 7.5
    static constexpr double k_IntakeHorizontalRollerSpeed = 1;
    static constexpr double k_IntakeVerticalRollerSpeed = 1;
    static constexpr double k_IntakeBarfHorizontalRollerSpeed = -1;
    static constexpr double k_IntakeBarfVerticalRollerSpeed = -1;
    static constexpr double k_IntakeHorizontalCurrentLimit = 0;
    static constexpr double k_IntakeVerticalCurrentLimit = 0;
    static constexpr double k_IntakeCurrentDuration = 0;
}
namespace FeederConstants{ //TODO figure out
    static constexpr double kDefaultFeederSpeed = .7;
    static constexpr double kShootingSpeed = .47;
    static constexpr double kIndexerSpeed = 1.0;
    static constexpr double kShootingIndexerSpeed = 0.47/0.7;
}

namespace LimeLightConstants{
    static constexpr double k_MarkersPipeline = 0;
    static constexpr double k_TopPostPipeLine = 1;
    static constexpr double k_BottomPostPipeLine = 2;
    static constexpr double k_TopPostHeight_in = 0;
    static constexpr double k_BottomPostHeight_in = 0;
    static constexpr double k_CameraHeight_in = 0;
    static constexpr double k_AprilTagHeight_in = 0;
}

namespace ElevatorConstants{
    static constexpr double k_ElevatorkP = 0;
    static constexpr double k_ElevatorkI = 0;
    static constexpr double k_ElevatorkD = 0;
    static constexpr double k_ElevatorkF = 0;
    static constexpr double k_ElevatorAcceleration = 0; //36000
    static constexpr double k_ElevatorMaxSpeed = 0; //24000
    static constexpr double k_ElevatorSCurveStrength = 6;
    static constexpr double k_ElevatorTicksPerInch = 0;
    static constexpr double k_ElevatorTopSoftLimit = 0;
    static constexpr double k_ElevatorBottomSoftLimit = 0;
    static constexpr double k_ElevatorOnTargetThreshold = 0;
    static constexpr double k_ElevatorTopPostPosition = 0;
    static constexpr double k_ElevatorBottomPostPosition = 0;
    static constexpr double k_ElevatorFloorPosition = 0;
    static constexpr double k_ElevatorMidShelfPosition = 0;
    static constexpr double k_ElevatorTopShelfPosition = 0;
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
    static constexpr double k_wheelRad = (4.25 / 2) *.0254; // in TODO find actual size
    static constexpr double k_maxSpeed = 9001; //TODO change also in driveWithJoystickCommand
    static constexpr double k_maxAccel = 1;
    static constexpr double k_maxDeccel = 1;
    static constexpr double k_steerEncoderToWheelGearRatio = 1; 
    static constexpr double k_driveMotorGearRatio = (11.0/30.0)*(1.0/3.0);
    static constexpr double k_ticksPerRev= 2048.0;//ticks per 100ms TODO check
    static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;
    static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(k_driveMotorGearRatio)*k_wheelRad*3.14159265*2;
    static constexpr double k_minRobotVelocity = 1;
    static constexpr double k_minRobotYawRate = 1;
    static constexpr double k_driveWheelSlotError = 1;
    static constexpr double k_robotWidth = 1;
    static constexpr double k_robotLength = 1;
    static constexpr double k_maxYawRate = k_maxSpeed / k_wheelLeverArm *180/std::numbers::pi;
    static constexpr double k_maxYawAccel = k_maxAccel / k_wheelLeverArm*180/std::numbers::pi;
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/std::numbers::pi;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180/std::numbers::pi;
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
    static constexpr double k_wheelRad = (3.79/2)*.0254*1.03; // in TODO find actual size
    static constexpr units::feet_per_second_t k_maxSpeed = units::feet_per_second_t(15); //TODO change also in driveWithJoystickCommand
    static constexpr double k_maxAccel = 3;
    static constexpr double k_maxDeccel = 1;
    static constexpr double k_steerEncoderToWheelGearRatio = 1; 
    static constexpr double k_driveMotorGearRatio = (11.0/30.0)*(1.0/3.0);
    static constexpr double k_ticksPerRev= 2048.0;//ticks per 100ms TODO check
    static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;
    static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2;
    static constexpr double k_minRobotVelocity = 1;
    static constexpr double k_minRobotYawRate = 1;
    static constexpr double k_driveWheelSlotError = 1;
    static constexpr double k_robotWidth = 1;
    static constexpr double k_robotLength = 1;
    static constexpr units::radians_per_second_t k_maxYawRate = units::radians_per_second_t(k_maxSpeed.value() / k_wheelLeverArm);
    static constexpr units::radians_per_second_squared_t k_maxYawAccel = units::radians_per_second_squared_t(k_maxAccel / k_wheelLeverArm);
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/std::numbers::pi;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180/std::numbers::pi;
    static constexpr double k_yawKp = 0;
    static constexpr double k_yawKi = 0;
    static constexpr double k_yawKd = 0;
    static constexpr double k_xyKp = 0;
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
#define RobotParameters RobotParametersTest

#endif // ROBOT_PARAMETERS_H