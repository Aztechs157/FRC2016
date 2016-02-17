
package org.usfirst.frc157.FRC2016;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 *
 * @author matt
 */
public class JoystickAxisButton extends Button
{
	
	Joystick m_controller;
	int m_buttonNumber;
	
	/**
	 * Create a joystick button for triggering commands
	 *
	 * @param joystick
	 *            The joystick object that has the button
	 * @param buttonNumber
	 *            The button number (see {@link Joystick#getRawButton(int) }
	 */
	public JoystickAxisButton(Joystick joystick, int axisNumber)
	{
		m_controller = joystick;
		m_buttonNumber = axisNumber;
	}
	
	/**
	 * Gets the value of the joystick button
	 *
	 * @return The value of the joystick button
	 */
	@Override
	public boolean get()
	{
		return (m_controller.getY() > 0.9);
	}
}
