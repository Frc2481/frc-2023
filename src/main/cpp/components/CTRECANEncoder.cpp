#include "components/CTRECANEncoder.h"
#include <sstream>
#include <ctre/phoenix/motorcontrol/FeedbackDevice.h>
#include <frc/Preferences.h>
#include "utils/NormalizeToRange.h"
#include <units/constants.h>

CTRECANEncoder::CTRECANEncoder(int CANID, const std::string &name)
	:
    m_pCANCoder(new CANCoder(CANID)),
    m_encoderTicks(0),
    m_encoderTicksZero(0)
     {
    std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();
    m_encoderTicksZero = frc::Preferences::GetDouble(m_calibrationKey);
    printf("\n\n\nEncoder ticks zero %d\n\n\n\n", m_encoderTicksZero);
    m_pCANCoder->ConfigSensorInitializationStrategy(ctre::phoenix::sensors::BootToAbsolutePosition);
    m_pCANCoder->ConfigFeedbackCoefficient(1, "ticks", ctre::phoenix::sensors::Per100Ms_Legacy);
    m_pCANCoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20, 10);
    m_pCANCoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 20, 10);
    m_pCANCoder->SetPositionToAbsolute();

    //m_CANCoder->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
    //m_CANCoder->SetStatusFramePeriod(Status_2_Feedback0, 10, 10);
   // m_CANCoder->
}

CTRECANEncoder::~CTRECANEncoder() {
}

void CTRECANEncoder::update() {
    m_encoderTicks = m_pCANCoder->GetPosition();
}

void CTRECANEncoder::zero() {
    m_encoderTicksZero = m_pCANCoder->GetPosition();
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

int CTRECANEncoder::getTicks() const {
	return m_encoderTicks - m_encoderTicksZero;
}

double CTRECANEncoder::getRevs() const {
    return getTicks() / 4096.0; //ticksperrev
}

double CTRECANEncoder::getAngle() const {
    return normalizeToRange::NormalizeToRange(std::fmod(getRevs(), 1) * 360.0, -180, 180, true);
}

double CTRECANEncoder::getWheelDistance(double wheelRadius, double gearRatioEncoderToWheel) const {
    return getRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * units::constants::pi; 
}

int CTRECANEncoder::convertRevsToTicks(double revs) const {
    return revs * 4096; //ticksperrev
}

int CTRECANEncoder::convertRevsToTickSetpoint(double revs) const {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

int CTRECANEncoder::convertAngleToTicks(double angle) const {
    return convertRevsToTicks(angle / 360.0);
}

int CTRECANEncoder::convertAngleToTickSetpoint(double angle) const {
    double error = normalizeToRange::RangedDifference(angle - getAngle(), -180, 180);
    return getTicks() + convertAngleToTicks(error) + m_encoderTicksZero;
}

double CTRECANEncoder::convertWheelDistanceToRevs(double wheelRadius, double wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * units::constants::pi); 
}

int CTRECANEncoder::convertWheelDistanceToTicks(double wheelRadius, double wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelRadius, wheelDistance));
}

int CTRECANEncoder::convertWheelDistanceToTickSetpoint(double wheelRadius, double wheelDistance) const {
    return convertWheelDistanceToTicks(wheelRadius, wheelDistance) + m_encoderTicksZero;
}

bool CTRECANEncoder::isConnected() const {
	return true;
}

bool CTRECANEncoder::isCalibrated() const {
	return fabs(m_encoderTicksZero) > 0;
}

int CTRECANEncoder::getZero() const {
    return m_encoderTicksZero;
}
CANCoder* CTRECANEncoder::getCANCoder() {
    return m_pCANCoder;
}