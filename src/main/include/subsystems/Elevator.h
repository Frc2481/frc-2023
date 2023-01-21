#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

class Elevator : public frc2::SubsystemBase {
public:
    Elevator();

    frc2::CommandPtr GoToBottomPostCommand();
    frc2::CommandPtr GoToTopPostCommand();
    frc2::CommandPtr GoToPositionCommand(double pos);
    frc2::CommandPtr WaitForElevatorOnTargetCommand();
    void Periodic() override;

    void SetTargetPosition(double pos);

    double GetTargetPosition();

    double GetActualPosition();

    void Zero();

    bool IsOnTarget();
        
private:

 TalonFX* m_pMotor;
 double m_desiredPosition;
};