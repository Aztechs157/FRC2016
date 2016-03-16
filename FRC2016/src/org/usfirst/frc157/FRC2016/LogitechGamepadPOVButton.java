package org.usfirst.frc157.FRC2016;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

//
//  This class allows the POV axis on sticks that support it to be used as a button
//
//  e.g. the Logitech 3d reports the angle of the POV stick in 45 degree increments
//        corresponding to to the push directions around the POV hat button
//

public class LogitechGamepadPOVButton  extends Button{

	private LogitechController controller;
	private int povAngle;
	
	  /**
	   * Enable POV stick as a set of buttons based on direction
	   *
	   * @param controller - the stick with the axis to use as a button
	   * @param angle - POV stick angle to treat as a button press (e.g. 0,45,90,135 etc...) 
	   **/
	LogitechGamepadPOVButton(LogitechController controller, int povAngle)
	{
		this.controller = controller;
		this.povAngle = povAngle; 				
	}

	/**
	 * Gets the value of the joystick button
	 *
	 * @return The value of the joystick button
	 */
	@Override
	public boolean get() {
		return (controller.getPOV() == povAngle);
	}

}
