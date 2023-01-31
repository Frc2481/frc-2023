// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "components/CTREMagEncoder.h"
#include <sstream>
#include <ctre/phoenix/motorcontrol/FeedbackDevice.h>
#include <frc/Preferences.h>
#include "Utils/NormalizeToRange.h"
#include "units/constants.h"
 

CTREMagEncoder::CTREMagEncoder(CommonMotorController* pTalon, const std::string &name)
	:
    m_encoderTicks(0),
    m_encoderTicksZero(0) {
    m_pTalon = pTalon;
    std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();
    m_encoderTicksZero = frc::Preferences::GetDouble(m_calibrationKey);
    printf("\n\n\nEncoder ticks zero %d\n\n\n\n", m_encoderTicksZero);

    m_pTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
    m_pTalon->SetStatusFramePeriod(Status_2_Feedback0, 10, 10);
    
}

CTREMagEncoder::~CTREMagEncoder() {
}

void CTREMagEncoder::update() {
    m_encoderTicks = m_pTalon->GetSelectedSensorPosition(0);
}

void CTREMagEncoder::zero() {
    m_encoderTicksZero = m_pTalon->GetSelectedSensorPosition(0);
    while(m_encoderTicksZero < 0){
        printf(" m_endoderTicks: %d\n", m_encoderTicksZero);
        m_encoderTicksZero += 4096;
    }
    if(m_encoderTicksZero >= 4096){
        m_encoderTicksZero %= 4096;
    }
    frc::Preferences::SetDouble(m_calibrationKey, m_encoderTicksZero);
    printf(" Encoder %s finalized: %d\n",m_calibrationKey.c_str(), m_encoderTicksZero);
    
}

int CTREMagEncoder::getTicks() const {
	return m_encoderTicks - m_encoderTicksZero;
}

double CTREMagEncoder::getRevs() const {
    return getTicks() / 4096.0; //ticksperrev
}

double CTREMagEncoder::getAngle() const {
    return normalizeToRange::NormalizeToRange(std::fmod(getRevs(), 1) * 360.0, -180, 180, true);
}

double CTREMagEncoder::getWheelDistance(double wheelRadius, double gearRatioEncoderToWheel) const {
    return getRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * units::constants::pi; 
}

int CTREMagEncoder::convertRevsToTicks(double revs) const {
    return revs * 4096; //ticksperrev
}

int CTREMagEncoder::convertRevsToTickSetpoint(double revs) const {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

int CTREMagEncoder::convertAngleToTicks(double angle) const {
    return convertRevsToTicks(angle / 360.0);
}

int CTREMagEncoder::convertAngleToTickSetpoint(double angle) const {
    double error = normalizeToRange::RangedDifference(angle - getAngle(), -180, 180);
    return getTicks() + convertAngleToTicks(error) + m_encoderTicksZero;
}

double CTREMagEncoder::convertWheelDistanceToRevs(double wheelRadius, double wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * units::constants::pi); 
}

int CTREMagEncoder::convertWheelDistanceToTicks(double wheelRadius, double wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelRadius, wheelDistance));
}

int CTREMagEncoder::convertWheelDistanceToTickSetpoint(double wheelRadius, double wheelDistance) const {
    return convertWheelDistanceToTicks(wheelRadius, wheelDistance) + m_encoderTicksZero;
}

bool CTREMagEncoder::isConnected() const {
	return m_pTalon->isSensorConnected() > 0;
}

bool CTREMagEncoder::isCalibrated() const {
	return fabs(m_encoderTicksZero) > 0;
}

int CTREMagEncoder::getZero() const {
    return m_encoderTicksZero;
}
