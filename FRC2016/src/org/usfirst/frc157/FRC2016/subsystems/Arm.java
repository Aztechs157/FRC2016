// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2016.subsystems;

import org.usfirst.frc157.FRC2016.RobotMap;
import org.usfirst.frc157.FRC2016.commands.*;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Arm extends Subsystem {

	public enum Position
	{
		FULL_DOWN           (-20.0),  // Minimum Shoulder Angle
		FRENCH_FRIES_DOWN   (-15.0),  // Push down fries to this Angle before crossing
		GRAB_BOULDER        (-12.0),  // Angle to use to grab a boulder
		LOW_BAR_TRAVEL      (-10.0),  // Angle to use to go under low bar
		PREPARE_FOR_BOULDER (10.0),   // Normal Bar Angle (ready to grab boulder)
		HOME                (0.0),    // Starting Angle is defined as 0, all other angles are referenced to it ~30 degrees
		GAME_START          (0.0),    // Game Start Angle (arm is inside legal starting box) 
		DRAWBRIDGE_GRAB     (20.0),   // Angle required to start Drawbridge Grab
		TOWER_SCALE         (50.0),   // Angle to use to extend ladder to scale tower
		FULL_UP             (70.0);   // Maximum shoulder Angle

		private final double angle;  // in degrees

		Position(double angle) {
			this.angle = angle;
		}
		
		double angle()
		{
			// returns the angle associated with the position
			return this.angle;
		}
	}
	
	public enum ArmMode
	{
		AUTOMATIC,
		MANUAL;
	}

	private class SensorCal
	{
		int zeroAngleReading;
		int maxAngleReading;
		double maxAngleDegrees;
		
		double readingsPerDegree;
		
		SensorCal(int zeroReading, int maxReading, double maxDegrees)
		{
			 zeroAngleReading = zeroReading;
			 maxAngleReading = maxReading;
			 maxAngleDegrees = maxDegrees;
			
			 double readingsPerDegree = ((double)(maxAngleReading - zeroAngleReading))/(maxDegrees);
		}
		
		public double readingToDegrees(double reading)
		{
			return (reading - zeroAngleReading) *  readingsPerDegree;
		}
	}
	
	private SensorCal cal = new SensorCal(350, 700, 90.0);
	
	private static final double compensationVoltageAtZero = 6.0; //Volts
		
	private ArmMode armMode;
	private ArmMode extenderMode;
	
	// TODO fix Arm subsystem to use TALON itegrated control loop instead of roboRio control loop
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final CANTalon shoulderMotor = RobotMap.armShoulderMotor;
    private final CANTalon extenderMotorA = RobotMap.armExtenderMotorA;
    private final CANTalon extenderMotorB = RobotMap.armExtenderMotorB;

    private DigitalInput shoulderHomeSwitch = new DigitalInput(RobotMap.ArmHomeSwitchDigitalInID);
    
    
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Initialize your subsystem here
    public Arm() {
    	setShoulderMode(ArmMode.AUTOMATIC);
    	setExtenderMode(ArmMode.AUTOMATIC);
    }

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
     
    // TODO finish this code...
    public boolean setShoulderMode(ArmMode mode)
    {
    	switch(mode)
    	{
    	case MANUAL:
    		if(armMode != ArmMode.MANUAL)
    		{
    			armMode = ArmMode.MANUAL;
    			shoulderMotor.changeControlMode(TalonControlMode.PercentVbus);
    		}
    		break;
    	case AUTOMATIC:
    	default:           // Unknown modes will be treated as automatic
    		if(armMode != ArmMode.MANUAL)
    		{
    			armMode = ArmMode.AUTOMATIC;
    			shoulderMotor.changeControlMode(TalonControlMode.Position);
    			shoulderMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    		}
    		break;
    	}
    	return true;
    }
    
    public boolean setExtenderMode(ArmMode mode)
    {
    	switch(mode)
    	{
    	case MANUAL:
    		if(extenderMode != ArmMode.MANUAL)
    		{
    			extenderMode = ArmMode.MANUAL;
    		}
    		break;
    	case AUTOMATIC:
    	default:           // Unknown modes will be treated as automatic
    		if(extenderMode != ArmMode.AUTOMATIC)
    		{
    			extenderMode = ArmMode.AUTOMATIC;
    		}
    		break;
    	}
    	extenderMotorB.changeControlMode(CANTalon.TalonControlMode.Follower);
       	extenderMotorB.set(extenderMotorA.getDeviceID());
  
    	return true;
    }

    public boolean shoulderSetVoltage(double voltage)
    {
    	setExtenderMode(ArmMode.MANUAL);
    	return true;
    }

    public boolean getShoulderHomeSwitch()
    {
    	return shoulderHomeSwitch.get();
    }
    
    public boolean shoulderAngleOK()
    {
    	// if the shoulder angle is above the high limit 
    	//      return false
    	// if the shoulder angle is tbelow the low limit
    	//      return false
    	return true;
    }
       
    public boolean shoulderGotoAngle(double angle)
    {
    	setExtenderMode(ArmMode.AUTOMATIC);
    	return true;
    }
    
    public boolean shoulderGotoPosition(Position position)
    {
    	return shoulderGotoAngle(position.angle());
    }
    
    public boolean armExtend()
    {
    	// run arm out until it reaches the end
    	return true;
    }
    
    public boolean armRetract()
    {
    	// run arm in until it reaches the end
    	return true;
    }
    
    public double getShoulderAngle()
    {
    	return cal.readingToDegrees(shoulderMotor.getAnalogInRaw());    	
    }
    
    public double getShoulderGravityCompensation()
    {
    	double compensation = Math.cos(getShoulderAngle()) * compensationVoltageAtZero;
    	return compensation;
    }
    
    private class ShoulderTask implements Runnable
    {
    	double output;
    	boolean stop;
    	boolean enableOutput;
    	
		@Override
		public void run() {
			// TODO Auto-generated method stub
			stop = false;
			while(!stop)
			{
				output = 0;				
				if(enableOutput)
				{
					output += getShoulderGravityCompensation();
					shoulderMotor.set(output);
				}
			}
		}

		public void enable()
		{
			enableOutput = true;
		}
		public void disable()
		{
			enableOutput = false;
		}
    }
}
