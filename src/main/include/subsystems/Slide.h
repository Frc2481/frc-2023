#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

class Slide : public frc2::SubsystemBase {
public:
    Slide();

    frc2::CommandPtr GoToCenterPositionCommand();
    frc2::CommandPtr TrackLimelightTopPostCommand();
    frc2::CommandPtr TrackLimelightBottomPostCommand();
    frc2::CommandPtr GoToPositionCommand(double pos);
    frc2::CommandPtr WaitForSlideOnTargetCommand();
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