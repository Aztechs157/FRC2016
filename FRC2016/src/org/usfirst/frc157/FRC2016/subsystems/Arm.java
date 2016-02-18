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

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.RobotMap;
import org.usfirst.frc157.FRC2016.commands.*;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class Arm extends Subsystem {

	public static final double EXTEND_SPEED = 0.5;
	public static final double RETRACT_SPEED = -0.5;
	public static final double STOP = 0;
		
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
	private final Timer time = RobotMap.time;
	
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
	private static ShoulderTask shoulder;
	
	// TODO fix Arm subsystem to use TALON itegrated control loop instead of roboRio control loop
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final CANTalon shoulderMotor = RobotMap.armShoulderMotor;
    private final CANTalon extenderMotorA = RobotMap.armExtenderMotorA;
    private final CANTalon extenderMotorB = RobotMap.armExtenderMotorB;

    private DigitalInput shoulderHomeSwitch = new DigitalInput(RobotMap.ArmHomeSwitchDigitalInID);
    private DigitalInput armExtendedSwitch = new DigitalInput(RobotMap.ArmExtendedSwitchDigitalInID);
    private DigitalInput armRetractedSwitch = new DigitalInput(RobotMap.ArmRetractedSwitchDigitalInID);
    
    
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Initialize your subsystem here
    public Arm() {
    	setShoulderMode(ArmMode.AUTOMATIC);
    	setupExtenderMotors(ArmMode.AUTOMATIC);

    	// if we haven't already started the shoulder task;
    	if(shoulder == null)
    	{
    		shoulder = new ShoulderTask(this);
    		shoulderTask = new Thread(shoulder);
    		shoulderTask.setDaemon(true);
    		shoulderTask.start();
    	}

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
    			shoulderMotor.setFeedbackDevice(FeedbackDevice.AnalogPot);
    		}
    		break;
    	}
    	return true;
    }
    
    public boolean extendAllowedByTime(){
     if (time.getMatchTime()>=130){
         return true;
     }
     else{
         return false;
     }
    }
    
    public boolean setupExtenderMotors(ArmMode mode)
    {
    	extenderMotorA.enableBrakeMode(true);
    	extenderMotorB.enableBrakeMode(true);  //TODO: Does the follower need to be set to brake?
    	
    	extenderMotorB.changeControlMode(CANTalon.TalonControlMode.Follower);
       	extenderMotorB.set(extenderMotorA.getDeviceID());
  
    	return true;
    }

    public boolean shoulderSetVoltage(double voltage)
    {
    	setupExtenderMotors(ArmMode.MANUAL);
    	shoulderMotor.set(voltage);
    	return true;
    }

    public boolean getShoulderHomeSwitch()
    {
    	return shoulderHomeSwitch.get();
    }
    
    public boolean getArmExtendedSwitch()
    {
    	return armExtendedSwitch.get();
    }
    
    public boolean getArmRetractedSwitch()
    {
    	return armRetractedSwitch.get();    	
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
    	setupExtenderMotors(ArmMode.AUTOMATIC);
    	shoulder.setTargetAngle(angle);
    	return true;
    }
    
    public boolean shoulderGotoPosition(Position position)
    {
    	return shoulderGotoAngle(position.angle());
    }
    
    public boolean armExtend()
    {
    	// run arm out until it reaches the end
    	// return true if the end is reached
    	if(getArmExtendedSwitch() == false)
    	{
    		extenderMotorB.set(EXTEND_SPEED);
    		return true;
    	}
    	else
    	{
    		extenderMotorB.set(STOP); 
    		return false;
    	}
    }
    
    public boolean armRetract()
    {
    	// run arm in until it reaches the end
    	// return true if the end is reached
    	if(getArmRetractedSwitch() == false)
    	{
    		extenderMotorB.set(RETRACT_SPEED);
    		return false;
    	}
    	else
    	{
    		extenderMotorB.set(STOP); 
    		return true;
    	}
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
    
    private Thread shoulderTask;
    private class ShoulderTask implements Runnable
    {

    	public static final double CONTROL_P = 0.125;
    	public static final double CONTROL_I = 0.05;
    	public static final double INTEGRAL_LIMIT = 5.0; // degrees

    	Arm arm;
    	double output;
    	boolean stop = false;
    	boolean enableOutput;
    	double targetAngle;
    	
    	public ShoulderTask(Arm parent)
    	{
    		arm = parent;
    		targetAngle = 0;
    		enableOutput = false;
    		output = 0;
    	}
    	    	
    	public void setTargetAngle(double angle)
    	{
    		if(angle < Position.FULL_DOWN.angle())
    		{
    			angle = Position.FULL_DOWN.angle();
    		}
    		if(angle > Position.FULL_UP.angle())
    		{
    			angle = Position.FULL_UP.angle();
    		}
    		targetAngle = angle;
    	}
    	
		@Override
		public void run() {
			double error;
			double sumError = 0;
			stop = false;
			
			////////////////////////////////////////////////////////////////////////////////////////
			/// TASK LOOP //////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////
			while(!stop)
			{
				error = targetAngle - getShoulderAngle();

				// This is not a real PI loop, but it is similar
				//
				//  We are proportional to our angular error but our error integration term
				//  is only added when we are close to the target angle (and zeroed when outside)
				//  so that we can avoid winding up the error term and having the arm swing wildly.
				//
				//  Additionally, we add a term to compensate for gravity independently of the
				//  control loop to keep the control loop from trying to compensate for the gravity
				//  and accelerating rapidly as the arm rises.
				
				// INTEGRAL Term
				/// build up the error sum term (integral) when the angle is close
				if(Math.abs(error) < INTEGRAL_LIMIT)
				{
					sumError += CONTROL_I * error;
				}
				else  // zero the term when we are further away
				{
					sumError = 0;
				}
				
				// PROPORTIONAL term
				// output is proportional to the error and the integrated (summed) error
				output = CONTROL_P * (error + sumError);				
				
				if(enableOutput)
				{
					output += getShoulderGravityCompensation();
					shoulderMotor.set(output);
				}
			}
			////////////////////////////////////////////////////////////////////////////////////////
			/// END TASK LOOP //////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////
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
