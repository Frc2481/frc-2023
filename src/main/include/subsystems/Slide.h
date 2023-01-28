#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>

class Slide : public frc2::SubsystemBase {
public:
    Slide();

    frc2::InstantCommand GoToCenterPositionCommand();
    frc2::FunctionalCommand TrackLimelightTopPostCommand();
    frc2::FunctionalCommand TrackLimelightBottomPostCommand();
    frc2::InstantCommand GoToPositionCommand(double pos);
    frc2::WaitUntilCommand WaitForSlideOnTargetCommand();
    frc2::FunctionalCommand TrackAprilTagsMidShelfCommand();
    frc2::FunctionalCommand TrackAprilTagsTopShelfCommand();
    frc2::WaitUntilCommand WaitForTargetVisibleCommand();

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