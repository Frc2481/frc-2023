#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>


class Elevator : public frc2::SubsystemBase {
public:
    Elevator();

    frc2::InstantCommand StowCommand();
    frc2::InstantCommand GoToFloorCommand();
    frc2::InstantCommand GoToBottomPostCommand();
    frc2::InstantCommand GoToTopPostCommand();
    frc2::InstantCommand GoToMidShelfCommand();
    frc2::InstantCommand GoToTopShelfCommand();
    frc2::InstantCommand GoToPositionCommand(double pos);
    frc2::WaitUntilCommand WaitForElevatorOnTargetCommand();
    frc2::InstantCommand EngageBrakeCommand();
    frc2::InstantCommand ReleaseBrakeCommand();

    void Periodic() override;

    void SetTargetPosition(double pos);

    double GetTargetPosition();

    double GetActualPosition();

    void Zero();

    bool IsOnTarget();

    void EngageBrake();

    void ReleaseBrake();
        
private:

 TalonFX* m_pMotor;
 double m_desiredPosition;
 frc::DoubleSolenoid * m_brakeSolenoid;
 frc::DigitalInput * m_elevatorBeambreak;
};