
package org.usfirst.frc157.FRC2016;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 *
 * @author Jarrod Risley
 */
public class LogitechController extends GenericHID
{
	
	// Define Enumeration of Valid Axes
	public static class AxisType
	{
		
		public final int value;
		public static final AxisType kAxisLeftX = new AxisType(m_kAxisLeftX);
		public static final AxisType kAxisLeftY = new AxisType(m_kAxisLeftY);
		public static final AxisType kAxisRightX = new AxisType(m_kAxisRightX);
		public static final AxisType kAxisRightY = new AxisType(m_kAxisRightY);
		public static final AxisType kAxisGameX = new AxisType(m_kAxisGameX);
		public static final AxisType kAxisGameY = new AxisType(m_kAxisGameY);
		public static final AxisType kAxisTriggers = new AxisType(m_kAxisTriggers);
		
		private AxisType(int axisNumber)
		{
			value = axisNumber;
		}
	}
	
	// Define Enumeration of Valid Buttons
	public static class ButtonType
	{
		
		public final int value;
		public static final ButtonType kButtonLeftB = new ButtonType(m_kButtonLeftB);
		public static final ButtonType kButtonRightB = new ButtonType(m_kButtonRightB);
		public static final ButtonType kButtonLeftTop = new ButtonType(m_kButtonLeftTop);
		public static final ButtonType kButtonRightTop = new ButtonType(m_kButtonRightTop);
		public static final ButtonType kButtonA = new ButtonType(m_kButtonA);
		public static final ButtonType kButtonB = new ButtonType(m_kButtonB);
		public static final ButtonType kButtonX = new ButtonType(m_kButtonX);
		public static final ButtonType kButtonY = new ButtonType(m_kButtonY);
		public static final ButtonType kButtonStart = new ButtonType(m_kButtonStart);
		public static final ButtonType kButtonBack = new ButtonType(m_kButtonBack);
		public static final ButtonType kButtonLeftTrigger = new ButtonType(m_kPseudoButtonLeftTrigger);
		public static final ButtonType kButtonRightTrigger = new ButtonType(m_kPseudoButtonRightTrigger);
		public static final ButtonType kButtonGameUp = new ButtonType(m_kPseudoButtonGameUp);
		public static final ButtonType kButtonGameDown = new ButtonType(m_kPseudoButtonGameDown);
		public static final ButtonType kButtonGameLeft = new ButtonType(m_kPseudoButtonGameLeft);
		public static final ButtonType kButtonGameRight = new ButtonType(m_kPseudoButtonGameRight);
		
		private ButtonType(int buttonNumber)
		{
			value = buttonNumber;
		}
	}
	
	double centerValue[] =
	{ 0, 0, 0, 0, 0, 0, 0, 0 };
	// Stick Axes (Note: Mode button switches LeftX between Gamepad and Stick
	private static final int m_kAxisLeftX = 1; // Left - Right
	private static final int m_kAxisLeftY = 2; // Up - Down
	private static final int m_kAxisRightX = 4; // Left - Right
	private static final int m_kAxisRightY = 5; // Up - Down
	// Stick Buttons
	private static final int m_kButtonLeftTop = 9;    
	private static final int m_kButtonRightTop = 10;
	// Gamepad Axes
	private static final int m_kAxisGameX = 6; // Left -
	private static final int m_kAxisGameY = 2; // Up -
	// Trigger Buttons (Note actual triggers LT, RT are acutally an Axis)
	private static final int m_kButtonLeftB = 5;
	private static final int m_kButtonRightB = 6;
	private static final int m_kAxisTriggers = 3; // Left Trigger +, Right Trigger -
	// Right Side Buttons
	private static final int m_kButtonA = 1;  //A button
	private static final int m_kButtonB = 2;  //B button
	private static final int m_kButtonX = 3;  //X button
	private static final int m_kButtonY = 4;  //Y button
	// Central Buttons
	private static final int m_kButtonStart = 8;
	private static final int m_kButtonBack = 7;
	// Pseudo Button (positve axes start at base, negative at 2 * base)
	private static final int m_kPseudoButtonBase = 1000;
	private static final int m_kPseudoButtonLeftTrigger = 2 * m_kPseudoButtonBase + m_kAxisTriggers;
	private static final int m_kPseudoButtonRightTrigger = m_kPseudoButtonBase + m_kAxisTriggers;
	private static final int m_kPseudoButtonGameUp = 2 * m_kPseudoButtonBase + m_kAxisGameY;
	private static final int m_kPseudoButtonGameDown = m_kPseudoButtonBase + m_kAxisGameY;
	
	private static final int m_kPseudoButtonGameLeft = 2 * m_kPseudoButtonBase + m_kAxisGameX;
	
	private static final int m_kPseudoButtonGameRight = m_kPseudoButtonBase + m_kAxisGameX;
	
	private DriverStation m_ds;
	private final int m_port;
	
	// by Thomas Pinella///Asim Conrad///Jarrod Risley
	double JsValueLeft;
	
	double JsValueRight;
	
	double JsDeadbandMidLimitUpper;
	
	double JsDeadbandMidLimitLower;
	
	double DesiredOutputValueMid;
	
	double JsDeadbandMaxLimit;
	double JsDeadbandMinLimit;
	double DesiredOutputValueMin;
	boolean isScaled = false;
	
	// Constructor
	public LogitechController(final int port)
	{
		m_ds = DriverStation.getInstance();
		m_port = port;
	}
	
	@Override
	public boolean getBumper(Hand hand)
	{
		return getRawButton(ButtonType.kButtonA.value);
	}
	
	// Buttons
	public boolean getButtonA()
	{
		return getRawButton(ButtonType.kButtonA.value);
	}
	
	public boolean getButtonB()
	{
		return getRawButton(ButtonType.kButtonB.value);
	}
	
	public boolean getButtonBack()
	{
		return getRawButton(ButtonType.kButtonBack.value);
		
	}
	
	public boolean getButtonGameDown()
	{
		return getRawButton(ButtonType.kButtonGameDown.value);
	}
	
	public boolean getButtonGameLeft()
	{
		return getRawButton(ButtonType.kButtonGameLeft.value);
	}
	
	public boolean getButtonGameRight()
	{
		return getRawButton(ButtonType.kButtonGameRight.value);
	}
	
	// Axis buttons
	public boolean getButtonGameUp()
	{
		return getRawButton(ButtonType.kButtonGameUp.value);
	}
	
	public boolean getButtonLB()
	{
		return getRawButton(ButtonType.kButtonLeftB.value);
	}
	
	public boolean getButtonLeftTop()
	{
		return getRawButton(ButtonType.kButtonLeftTop.value);
	}
	
	public boolean getButtonLeftTrigger()
	{
		return getRawButton(ButtonType.kButtonLeftTrigger.value);
	}
	
	public boolean getButtonRB()
	{
		return getRawButton(ButtonType.kButtonRightB.value);
	}
	
	public boolean getButtonRightTop()
	{
		return getRawButton(ButtonType.kButtonRightTop.value);
	}
	
	public boolean getButtonRightTrigger()
	{
		return getRawButton(ButtonType.kButtonRightTrigger.value);
	}
	
	public boolean getButtonStart()
	{
		return getRawButton(ButtonType.kButtonStart.value);
	}
	
	public boolean getButtonX()
	{
		return getRawButton(ButtonType.kButtonX.value);
	}
	
	public boolean getButtonY()
	{
		return getRawButton(ButtonType.kButtonY.value);
	}
	
	public double getGameX()
	{
		return getRawAxis(AxisType.kAxisGameX.value);
	}
	
	public double getGameY()
	{
		return getRawAxis(AxisType.kAxisGameY.value);
	}
	
	// Logitech Controller Specific Functions
	// Axes
	public double getLeftX()
	{
		return getRawAxis(AxisType.kAxisLeftX.value);
	}
	
	public double getLeftY()
	{
		return getRawAxis(AxisType.kAxisLeftY.value);
	}
	
	@Override
	public int getPOV(int pov)
	{
		// TODO Auto-generated method stub
		return 0;
	}
	
	// GenericHID Interface Functions
	// Hand is ignored for all functions
	@Override
	public double getRawAxis(int axisNum)
	{
		return m_ds.getStickAxis(m_port, axisNum) - centerValue[axisNum];
	}
	
	@Override
	public boolean getRawButton(int buttonNum)
	{
		if (buttonNum < m_kPseudoButtonBase)
		{
			// normal buttons
			return (0x1 << buttonNum - 1 & m_ds.getStickButtons(m_port)) != 0;
		}
		else if (buttonNum < 2 * m_kPseudoButtonBase)
		{
			// positive axis pseudo buttons
			return getRawAxis(buttonNum - m_kPseudoButtonBase) < -0.5;
		}
		else
		{
			// negative axis pseudo buttons
			return getRawAxis(buttonNum - 2 * m_kPseudoButtonBase) > 0.5;
		}
	}
	
	public double getRightX()
	{
		return getRawAxis(AxisType.kAxisRightX.value);
	}
	
	public double getRightY()
	{
		return getRawAxis(AxisType.kAxisRightY.value);
	}
	
	@Override
	public double getThrottle()
	{
		return getRawAxis(AxisType.kAxisGameX.value);
	}
	
	@Override
	public boolean getTop(Hand hand)
	{
		return getRawButton(ButtonType.kButtonLeftTop.value);
	}
	
	@Override
	public boolean getTrigger(Hand hand)
	{
		return getRawButton(ButtonType.kButtonLeftB.value);
	}
	
	public double getTriggers()
	{
		return getRawAxis(AxisType.kAxisTriggers.value);
	}
	
	@Override
	public double getTwist()
	{
		return getRawAxis(AxisType.kAxisRightY.value);
	}
	
	@Override
	public double getX(Hand hand)
	{
		return getRawAxis(AxisType.kAxisLeftX.value);
	}
	
	@Override
	public double getY(Hand hand)
	{
		return getRawAxis(AxisType.kAxisLeftY.value);
	}
	
	@Override
	public double getZ(Hand hand)
	{
		return getRawAxis(AxisType.kAxisRightX.value);
	}
	
	public void recenter(int axisNum)
	{
		centerValue[axisNum] = 0;
		centerValue[axisNum] = getRawAxis(axisNum);
		System.out.println("Axis " + axisNum + " recenter to " + centerValue[axisNum]);
	}
}
