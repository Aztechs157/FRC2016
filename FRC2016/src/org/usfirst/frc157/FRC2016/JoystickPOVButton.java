package org.usfirst.frc157.FRC2016;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class JoystickPOVButton  extends Button{

	private Joystick stick;
	private int povAngle;
	
	JoystickPOVButton(Joystick stick, int povAngle)
	{
		this.stick = stick;
		this.povAngle = povAngle; 				
	}
	@Override
	public boolean get() {
		return (stick.getPOV() == povAngle);
	}

}
