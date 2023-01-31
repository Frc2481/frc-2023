// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include "ctre/Phoenix.h"
#include "components/CommonMotorController.h"
class CTREMagEncoder {
public:
    CTREMagEncoder(CommonMotorController* pTalon, const std::string &name);
    ~CTREMagEncoder();

    void update();
    void zero();
    void zeroTalon();
    int getTicks() const;
    double getRevs() const;
    double getAngle() const;
    double getWheelDistance(double wheelRadius, double gearRatioEncoderToWheel) const;
    int convertRevsToTicks(double revs) const;
    int convertRevsToTickSetpoint(double revs) const;
    int convertAngleToTicks(double angle) const;
    int convertAngleToTickSetpoint(double angle) const;
    double convertWheelDistanceToRevs(double wheelRadius,  double wheelDistance) const;
    int convertWheelDistanceToTicks(double wheelRadius, double wheelDistance) const;
    int convertWheelDistanceToTickSetpoint(double wheelRadius, double wheelDistance) const;
    bool isConnected() const;
    bool isCalibrated() const;
    int getZero() const;

private:
    CommonMotorController* m_pTalon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_calibrationKey;
};