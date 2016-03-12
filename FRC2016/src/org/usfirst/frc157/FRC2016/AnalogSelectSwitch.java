package org.usfirst.frc157.FRC2016;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogSelectSwitch {
    
    public enum SwitchPosition
	{
		POSITION_0 (0.000,0.489, "Position 0"),  // will return this position if there is a problem
		POSITION_1 (0.489,1.466, "Position 1"),
		POSITION_2 (1.466,2.445, "Position 2"),
		POSITION_3 (2.445,3.425, "Position 3"),
		POSITION_4 (3.425,4.405, "Position 4"),
        POSITION_5 (4.405,5.000, "Position 5");
		
		private double rangeLowVoltage;
		private double rangeHighVoltage;
		public String postionName;
		
		SwitchPosition(double low, double high, String name)
		{
			rangeLowVoltage = low;
			rangeHighVoltage = high;
			postionName = name;
		}
	}
	
	AnalogInput switchInput;
	
	AnalogSelectSwitch(int analogInputPort)
	{
		if(switchInput == null)
		{
			switchInput = new AnalogInput(analogInputPort);
		}
	}
	
	public SwitchPosition getPosition()
	{
		double value = switchInput.getAverageVoltage();
		if((SwitchPosition.POSITION_1.rangeLowVoltage <= value) && (value < SwitchPosition.POSITION_1.rangeHighVoltage))
		{
			return SwitchPosition.POSITION_1;
		}
		else if((SwitchPosition.POSITION_2.rangeLowVoltage <= value) && (value < SwitchPosition.POSITION_2.rangeHighVoltage))
		{
			return SwitchPosition.POSITION_2;			
		}
		else if((SwitchPosition.POSITION_3.rangeLowVoltage <= value) && (value < SwitchPosition.POSITION_3.rangeHighVoltage))
		{
			return SwitchPosition.POSITION_3;
		}
		else if((SwitchPosition.POSITION_4.rangeLowVoltage <= value) && (value < SwitchPosition.POSITION_4.rangeHighVoltage))
		{
			return SwitchPosition.POSITION_4;			
		}
		else if((SwitchPosition.POSITION_5.rangeLowVoltage <= value) && (value < SwitchPosition.POSITION_5.rangeHighVoltage))
		{		
			return SwitchPosition.POSITION_5;			
		}
		else
		{
			return SwitchPosition.POSITION_0;
		}
			
	}
}
