
package org.usfirst.frc157.FRC2016;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class LogitechController extends Joystick
{
	
	protected LogitechController(int port, int numAxisTypes, int numButtonTypes) {
		super(port, numAxisTypes, numButtonTypes);
		// TODO Auto-generated constructor stub
	}

	  public LogitechController (final int port) {
		  super(port);
	  }


	  // Mode should be off
	  // Controller swithc should be set to X
	public enum AxisID
	{
		LEFT_STICK_X    (0),  // -1..1 + is right
		LEFT_STICK_Y    (1),  // -1..1 + is toward operator
		LEFT_TRIGGER    (2),  //  0..1 + is trigger more pulled
		RIGHT_TRIGGER   (3),  //  0..1 + is trigger more pulled
		RIGHT_STICK_X   (4),  // -1..1 + is right
		RIGHT_STICK_Y   (5);  // -1..1 + is toward operator
		private final int axisNum;  // in degrees

		AxisID(int axisNum) {
			this.axisNum = axisNum;
		}
		
		public int ID()
		{
			// returns the angle associated with the position
			return this.axisNum;
		}		
	}

	public enum ButtonID
	{
		A     (1),        // Action Button A
		B     (2),        // Action Button B
		X     (3),        // Action Button X
		Y     (4),	      // Action Button Y
		LEFT  (5),        // Left Button  (above left trigger)
		RIGHT (6),	      // Right Button (above right trigger)
		BACK  (7),        // Back Button  (between controller sides)
		START (8),        // Start Button (between controller sides)
		LEFT_STICK (9),   // Left Stick Button  (left stick click)  //FIXME - Verify Button Number
		RIGHT_STICK (10); // Right Stick Button (right stick click) //FIXME - Verify Button Number
		
		private final int buttonNum;
		
		ButtonID(int buttonNum) {
			this.buttonNum = buttonNum;
		}
		
		public int ID()
		{
			// returns the angle associated with the position
			return this.buttonNum;
		}
	}	
	
	public double getLeftX() 
	{
		return super.getRawAxis(AxisID.LEFT_STICK_X.ID());
	}
	public double getLeftY() 
	{
		return super.getRawAxis(AxisID.LEFT_STICK_Y.ID());
	}
	public double getRightX() 
	{
		return super.getRawAxis(AxisID.RIGHT_STICK_X.ID());
	}
	public double getRightY() 
	{
		return super.getRawAxis(AxisID.RIGHT_STICK_Y.ID());
	}
}
