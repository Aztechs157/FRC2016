
package org.usfirst.frc157.FRC2016;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

//
//This class allows the analog axis on sticks to be used as a button trigger
//  If the stick is pushed "far enough" the button is considered pressed
//  
public class JoystickAxisButton extends Button
{

	public enum Direction 
	{
		POSITIVE,
		NEGATIVE,
		BOTH;
	}

	private Joystick joystick;
	private int axisNumber;
	private Direction direction;
	private double triggerLevel;


	/**
	 * Allow a joystick axis to trigger a button event
	 *
	 * @param joystick - the stick with the axis to use as a button
	 * @param axisNumber - the axis on the stick to use as a button
	 * @param direction - allows for POSITIVE, NEGATIVE or BOTH directions to trigger the button press
	 * @param triggerLevel - axis value beyond which will trigger a button press (NEGATIVE direction should use negative numbers)
	 */

	public JoystickAxisButton(Joystick joystick, int axisNumber, Direction direction, double triggerLevel)
	{
		this.joystick = joystick;
		this.axisNumber = axisNumber;
		this.direction = direction;
		this.triggerLevel = triggerLevel;
	}

	/**
	 * Gets the value of the joystick button
	 *
	 * @return The value of the joystick button
	 */
	@Override
	public boolean get()
	{
		boolean result = false;

		if(!isPresent())
		{
			return false;
		}
		else
		{
			double axisPosition = joystick.getRawAxis(axisNumber);
			switch(direction)
			{
			case POSITIVE:  // stick must be more positive than triggerLevel
			{	
				if(axisPosition >= triggerLevel)
				{
					result = true;
				}
			}
			break;
			case NEGATIVE:  // stick must be more negative than triggerLevel
			{
				if(axisPosition <= triggerLevel)
				{
					result = true;
				}
			}
			case BOTH:     // stick must be at least triggerLevel away from the center (positive or negative)
			{
				if(Math.abs(axisPosition) > Math.abs(triggerLevel))
				{
					result = true;
				}
			}
			break;
			default:
			{
				result = false;
			}
			}
			return result;
		}
	}

	public boolean isPresent()
	{
		if((1 > joystick.getButtonCount()) && (1 > joystick.getAxisCount()))
		{
			return false;
		}
		else
		{			
			return true;
		}
	}


}
