// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "components/Joystick2481.h"
#include <math.h>
#include <frc/Joystick.h>

Joystick2481::Joystick2481(int port)
	: frc::Joystick(port) {	
}

Joystick2481::~Joystick2481() {
}

bool Joystick2481::GetAxis(int axis, float threshold){
	
	float input = frc::Joystick::GetRawAxis(axis);
	return threshold <= input;
}
float Joystick2481::GetRawAxis(int axis) {
	float input = frc::Joystick::GetRawAxis(axis);
	float scale = 1.0f;
	float deadband = 0.381021f;
	float slope = (1 -.059) /(1-.38);
	float offset = -(1 -.059) /(1-.38) +1;
	float second = 0.57f;

	// linear outside of deadband
    if (input < -deadband){
        return scale * (slope * input - offset);
    }
	// polynomial inside of deadband
    if ((-deadband < input) && (input < deadband)) {
        return scale * (1.0 / (pow(second, 2.0)) * pow(input, 3.0));
    }
	//linear outside of deadband
    else {
        return scale * (slope * input + offset);
    }

}