package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensor {
	
	AnalogInput input;
	public PressureSensor(int inputPin) {
		input = new AnalogInput(inputPin);
	}
	
	public int get() {
		double pressure = input.getVoltage() * 50.0 - 25.0;
		return (int) pressure;
	}
}