
#include <math.h>
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
// #include <units/units.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
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
    static constexpr int kIndexerMotorID = 10;
}

namespace FalconIDs{
    static constexpr int kFrontRightDriveMotorID = 3;//5
    static constexpr int kFrontLeftDriveMotorID = 1;//6
    static constexpr int kRearRightDriveMotorID = 4;//7
    static constexpr int kRearLeftDriveMotorID = 2;//8
    static constexpr int kRearMiddleDriveMotorID = 5;//TODO figure me out
    static constexpr int kTopShooterMotorID = 13; //TODO figure me out
    static constexpr int kBottomShooterMotorID = 12; //TODO figure me out
    static constexpr int kturretMotorID = 6; 
    static constexpr int kFloorClimberMotorID = 7; //TODO figure out
    static constexpr int kTrussClimberMotorID = 8; //TODO figure out
    static constexpr int kIntakeRollerMotorID = 9; //intake roller motor is a talon SRX
    
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

constexpr auto kMaxSpeed = units::meters_per_second_t(3);
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
    static constexpr double kDefaultIntakeRollerSpeed = 1; //TODO: Find out
    static constexpr double kIntakeCurrentBallDetectThreshhold = 23; //12; //TODO find out previous 7.5
}
namespace FeederConstants{ //TODO figure out
    static constexpr double kDefaultFeederSpeed = .7;
    static constexpr double kShootingSpeed = .47;
    static constexpr double kIndexerSpeed = 1.0;
    static constexpr double kShootingIndexerSpeed = 0.47/0.7;
}

namespace DigitalInputs{
    static constexpr int kFeederBeamBreakPort = 0;
    static constexpr int kIndexerBeamBreakPort = 1;
    static constexpr int kIntakeBeamBreakPort = 7;
    static constexpr int kFloorLineSensor = 5;
    static constexpr int kTrussLineSensor = 4;
}

namespace ShooterConstants{ //TODO wheel speeds
    static constexpr double kDonutTopWheelSpeed = 0; 
    static constexpr double kDonutBottomWheelSpeed = 0; 
    static constexpr double kTopShooterSpeed = -2500; //-2500
    static constexpr double kBottomShooterSpeed = -1700; // -1700


}

namespace LimelightConstants{
    static constexpr double kTargetHeight = 102.5; //inches
    static constexpr double kLimelightHeight = 45; //TODO find actual height
    static constexpr double kLimelightAngle = 19.5; //TODO find angle 21
    
}

namespace ClimberConstants{
    static constexpr double kFloorWheelSpeed = 1;
    static constexpr double kTrussWheelSpeed = 1;
    static constexpr double kJavelinDeployRoll = 45;
    static constexpr double kDriveTrainSpeedConstant = 0.4;
    
    
}

//TODO: figure this all out

namespace RobotParameters { 
    // robot main
	static constexpr unsigned k_updateRate = 50; // Hz

    static constexpr double k_wheelBase = 1; // in
    static constexpr double k_wheelTrack = 1; // in
    static constexpr double k_wheelLeverArm = sqrt(std::pow(k_wheelBase/2,2) + std::pow(k_wheelTrack/2,2));
    static constexpr double k_wheelRad = (3.79/2)*.0254*1.03; // in TODO find actual size
    static constexpr double k_maxSpeed = 9001; //TODO change also in driveWithJoystickCommand
    static constexpr double k_maxAccel = 1;
    static constexpr double k_maxDeccel = 1;
    static constexpr double k_steerEncoderToWheelGearRatio = 1; // gear ratio from steer encoder to wheel TODO find
    static constexpr double k_driveMotorGearRatio = 90.0/10.0;
    static constexpr double k_ticksPerRev= 2048.0;//ticks per 100ms TODO check
    static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;
    static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2;
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


    //shooter Constants TODO set correct values
    static constexpr double k_shooterBotP = 0.02;
    static constexpr double k_shooterBotI = 0.0;
    static constexpr double k_shooterBotD = 0.0;
    static constexpr double k_shooterBotF = 0.0542;

    static constexpr double k_shooterTopP = 0.1;
    static constexpr double k_shooterTopI = 0.0;
    static constexpr double k_shooterTopD = 0.0;
    static constexpr double k_shooterTopF = 0.0552;
    //turret Constants TODO set correct values
    // static constexpr double k_turretP = 1;
    // static constexpr double k_turretI = 0.0;
    // static constexpr double k_turretD = 0.0;
    // // static constexpr double k_turretF = (0.15 * 1023) / 1500;
    // static constexpr double k_turretF = .0775;
    static constexpr double k_turretP = .2; //.2
    static constexpr double k_turretI = 0; //.01
    static constexpr double k_turretD = 10;
    // static constexpr double k_turretF = (0.15 * 1023) / 1500;
    static constexpr double k_turretF = 0.0; //0.75

    static constexpr double k_turretAcceleration = 100000; //36000
    static constexpr double k_maxTurretSpeed = 16000; //24000
    static constexpr double k_turretEncoderTicksToDegrees = 360.0/2048.0; //Figure out Gear ratio
    static constexpr double k_turretEncoderTicksToDPS = 12.0*(360.0/2048.0);
    static constexpr double k_turretRadius = 6; 
    static constexpr double k_turretTeeth = 152; // 152.0;  This isn't right, but othere things are wrong so this makes it better.
    static constexpr double k_turretDriveTeeth = 1.6;
    static constexpr double k_turretGearRatio = k_turretTeeth/k_turretDriveTeeth;
    static constexpr double k_turretABSMaxRotations = k_turretGearRatio*2;
    static constexpr double k_turretADCPerRotation = 4096/k_turretABSMaxRotations;
    static constexpr double k_turretABSDegreesPerShaftRotation = 720.0/k_turretABSMaxRotations;
    static constexpr double k_turretTicksPerRotation = 2048*k_turretGearRatio;
    static constexpr double k_turretTicksPerNinetyDegrees = k_turretTicksPerRotation/4;
    static constexpr double k_turretTicksPerDegree = k_turretTicksPerRotation / 360;

    static constexpr double k_minTurretDegrees = -125;
    static constexpr double k_maxTurretDegrees = 225;

    static constexpr double k_minTurretSearchDegrees = -75; //-125
    static constexpr double k_maxTurretSearchDegrees = 75; //225


    //limelight TODO find corret values
    static constexpr double k_limeLightP = 4.9;
    static constexpr double k_limeLightI = 1;
    static constexpr double k_limeLightD = 0;
    static constexpr double k_limeLightIZone = 10;

    
}


#endif // ROBOT_PARAMETERS_H