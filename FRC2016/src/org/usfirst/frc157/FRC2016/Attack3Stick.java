
package org.usfirst.frc157.FRC2016;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Attack3Stick extends Joystick
{
	
	protected Attack3Stick(int port, int numAxisTypes, int numButtonTypes) {
		super(port, numAxisTypes, numButtonTypes);
		// TODO Auto-generated constructor stub
	}

	  public Attack3Stick (final int port) {
		  super(port);
	  }


	  // Mode should be off
	  // Controller swithc should be set to X
	public enum AxisID
	{
		STICK_X      (0),  // -1..1 + is right
		STICK_Y      (1),  // -1..1 + is toward operator
		THROTTLE     (2);  //  0..1 + is toward operator
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
		TRIGGER (1),   // Trigger
		TOP_2   (2),   // Top Button #2
		TOP_3   (3),   // Top Button #3
		TOP_4   (4),   // Top Button #4
		TOP_5   (5),   // Top Button #5
		TOP_6   (6),   // Base Button (Left) #6
		BASE_7  (7),   // Base Button (Left) #7
		BASE_8  (8),   // Base Button (Operator Side) #8
		BASE_9  (9),   // Base Button (Operator Side) #9
		BASE_10 (10),  // Base Button (Right) #10
		BASE_11 (11);  // Base Button (Right) #11
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
	
	public boolean isPresent()
	{
		if((1 > super.getButtonCount()) && (1 > super.getAxisCount()))
		{
			return false;
		}
		else
		{
			
			return true;
		}
	}
	
	public boolean isCorrect()
	{
		if(getName().equals("Logitech Attack 3"))
		{
			return true;
		}
		else
		{
			
			return false;
		}
	}	
}
