#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

class Elevator : public frc2::SubsystemBase {
public:
    Elevator();

    void Periodic() override;

    void SetDesiredPosition(double pos);

    double GetDesiredPosition();

    double GetActualPosition();

    void Zero();

    bool IsOnTarget();
        
private:

 TalonFX* m_pMotor;
};