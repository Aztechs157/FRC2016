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

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

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
       
    /////////////////////////////////////////////////////////////////////////
    ///
    ///      * * U P D A T E * *
    ///
    /// Polarity of Retract and Extend has been fixed.
    /// 
    /////////////////////////////////////////////////////////////////////////   

	public static final double EXTEND_SPEED = -0.3;  //Really Extend
	public static final double RETRACT_SPEED = 0.5;  //Really Retract
	public static final double CHAIN_BURP_SPEED = 0.25;
	public static final double CHAIN_BURP_TIME = 0.100; //seconds
	public static final double STOP = 0;
		
	public enum Position
	{
		FULL_DOWN           (-7.0),  // Minimum Shoulder Angle
		FRENCH_FRIES_DOWN   (-1.0),  // Push down fries to this Angle before crossing
		GRAB_BOULDER        (-1.0),  // Angle to use to grab a boulder
		LOW_BAR_TRAVEL      (0.0),  // Angle to use to go under low bar
		CLEAR_FOR_SHOT      (25.0),  //  Angle to move arm to to shoot
		PREPARE_FOR_BOULDER (10.0),   // Normal Bar Angle (ready to grab boulder)
		HOME                (0.0),    // Starting Angle is defined as 0, all other angles are referenced to it ~30 degrees
		GAME_START          (70.0),    // Game Start Angle (arm is inside legal starting box) 
		DRAWBRIDGE_GRAB     (20.0),   // Angle required to start Drawbridge Grab
		TOWER_SCALE         (90.0),   // Angle to use to extend ladder to scale tower
		FULL_UP             (90.0);   // Maximum shoulder Angle
		private final double angle;  // in degrees

		Position(double angle) {
			this.angle = angle;
		}
		
		public double angle()
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
			
			 readingsPerDegree = (maxDegrees)/((double)(maxAngleReading - zeroAngleReading));
			 System.out.println("Angle CAL: Zero     " + zeroAngleReading);
			 System.out.println("Angle CAL: Max Read " + maxAngleReading);
			 System.out.println("Angle CAL: Max Deg  " + maxAngleDegrees);
			 System.out.println("Angle CAL: Per Deg  " + readingsPerDegree);
		}
		
		public double readingToDegrees(int reading)
		{
			double angle;
			angle = ((double)reading - (double)zeroAngleReading) *  (double)readingsPerDegree;
//			System.out.println("Reading - " + reading + "  Angle - " + angle);
			return angle;
		}
		
		public void recalibrateLow(int lowReading)
		{
			
		}
		public void recalibrateHigh(int highReading)
		{
			
		}
		
		
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	// Calibrate the arm here
	//   Use the web page to get these values roborio-157.frc.local
	//   on talon 2 (the shoulder motor)
	//   use the self test button in each of these positions and enter the values below
	//   Position 1 - arm at 0 degrees (parallel to frame) is the first value below
	//   Position 2 - arm at 90 degrees (perpendicular to frame) is the second position
	//   Number three is the angle the second number was taken at
	//
	//  the number you want is the ADC value in the Analog section of the self test page
	//     don't forget to refresh...
	//
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	// Updated Mon 2/22/2016 after Week Zero axle repair
//	private SensorCal cal = new SensorCal(757, 239, 90.0);
	private SensorCal cal = new SensorCal(888, 387, 90.0);
	// Sensor cal is (888-837)/90 ticks per degree (5.567)
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

	private static final double compensationVoltageAtZero = 6.0; //Volts
		
	private ArmMode armMode;
	private static ShoulderTask shoulder;
	private Object shoulderAngleSemaphore = new Object();
	
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

//    	FileReader in = null;
//    	FileWriter out = null;
//    
//    	try
//    	{
////    		in = new FileReader("input.txt");
//    		// Apparently need full path...
//    		out = new FileWriter("/home/lvuser/output.txt");
//    		out.write("This is text");
//    		// wrote some txt
//    		out.close();
////    		in.close();
//    	}
//    	catch (IOException e)
//    	{
//    		System.out.println("Probably don't have the file input.txt");
//    	}
//    	finally
//    	{
//    		if(in != null) {
//    			try {
//					in.close();
//				} catch (IOException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//    		}
//    		if(out != null) {
//    			try {
//					out.close();
//				} catch (IOException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//    		}
//    	}
    }

    public void  enableControl()
    {
    	shoulder.enableControlOutput();
    }
    
    public void disableControl()
    {
    	shoulder.disableControlOutput();
    }
    
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
     
    public boolean setShoulderMode(ArmMode mode)
    {
//    	shoulderMotor.changeControlMode(TalonControlMode.Voltage);
    	shoulderMotor.changeControlMode(TalonControlMode.PercentVbus);
//    	shoulderMotor.enableBrakeMode(true);
		shoulderMotor.enable();

    	return true;
    }
    
    public boolean extendAllowedByTime(){
    	
    	// TODO REMOVE THE NEXT LINE
    	return true;
    	
//     if (time.getMatchTime()>=130){
//         return true;
//     }
//     else{
//         return false;
//     }
   }
    
    public boolean setupExtenderMotors(ArmMode mode)
    {
    	extenderMotorA.enableBrakeMode(true);
    	extenderMotorB.enableBrakeMode(true);  //TODO: Does the follower need to be set to brake?
    	
    	extenderMotorB.changeControlMode(CANTalon.TalonControlMode.Follower);
       	extenderMotorB.set(extenderMotorA.getDeviceID());
  
    	return true;
    }

    public void shoulderSetVoltage(double voltage)
    {
    	shoulderMotor.set(voltage);
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
    	System.out.println("\n\n+++++++++++++++++++++ Shoulder to: " + angle);
    	shoulder.enableControlOutput();
    	shoulder.setTargetAngle(angle);
    	shoulder.enableControlOutput();
    	return true;
    }
    
    public boolean shoulderGotoPosition(Position position)
    {
    	return shoulderGotoAngle(position.angle());
    }
    
    public boolean armRetract(double speed)
    {   
        //check speed polarity
        if (speed < 0) speed = -speed;
        
        // run arm out until it reaches the end
    	// return true if the end is reached
    	if(getArmRetractedSwitch() == true)
    	{
    		extenderMotorA.set(speed);
    		return true;
    	}
    	else
    	{
    		extenderMotorA.set(STOP); 
    		return false;
    	}
    }
       
    public boolean armExtend(double speed)
    {
        //check speed polarity
        if (speed > 0) speed = -speed;

    	// run arm in until it reaches the end
    	// return true if the end is reached
    	if(getArmExtendedSwitch() == true)
    	{
    		extenderMotorA.set(speed);
    		return false;
    	}
    	else
    	{
    		extenderMotorA.set(STOP); 
    		return true;
    	}
    }
    
    public void armExtendStop(){
        extenderMotorA.set(0);
    }

	static double lastAngleReading = 0;
	public double getShoulderAngle()
    {
    	int reading = -1;
    	synchronized(shoulderAngleSemaphore)
    	{
    		reading = shoulderMotor.getAnalogInRaw();
    	}
    	
    	double angle = cal.readingToDegrees(reading);
    	
    	// if reading is good, save it as the last reading
    	if((Position.FULL_DOWN.angle() <= angle) && (angle <= Position.FULL_UP.angle()))
    	{
    		lastAngleReading = angle;
    	}
    	// otherwise use the last reading as the reading and hope next time is better
    	else
    	{
    		angle = lastAngleReading;
    	}
    	
    	return cal.readingToDegrees(reading);    	
    }
    
    public double getShoulderGravityCompensation()
    {
    	double compensation = Math.cos(getShoulderAngle()) * compensationVoltageAtZero;
    	return compensation;
    }
    
    private Thread shoulderTask;
    private class ShoulderTask implements Runnable
    {

    	public static final double CONTROL_P = 0.75;
    	public static final double CONTROL_I = 0.0;//0.0125;
    	public static final double INTEGRAL_LIMIT = 5.0; // degrees
    	
    	public static final double MIN_OUTPUT = -12.0; // Volts
    	public static final double MAX_OUTPUT =  11.0; // Volts

    	Arm arm;
    	double output;
    	boolean stop = false;
    	volatile boolean enableOutput;
    	volatile double targetAngle;
    	
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
    		synchronized(arm)
    		{
    			targetAngle = angle;
    		}
        	System.out.println("\n\n+++++++++++++++++++++ TARGET   to: " + angle);

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
				/// slowly build up the error sum term (integral) when the angle is close
				if(Math.abs(error) < INTEGRAL_LIMIT)
				{
					sumError += CONTROL_I * error;
				}
				else  // zero the term when we are further away (prevents windup)
				{
					sumError = 0;
				}
				
				// PROPORTIONAL term
				// output is proportional to the error and the integrated (summed) error
				output = CONTROL_P * (error + sumError);				
				

//				output += getShoulderGravityCompensation();

//				System.out.print("t" + targetAngle + " a" + getShoulderAngle() + "  e" + error + "  s"+ sumError + "  o" + output);

				// Bound the output
				if(output < MIN_OUTPUT) { output = MIN_OUTPUT;}
				if(output > MAX_OUTPUT) { output = MAX_OUTPUT;}
				
//				System.out.print(" b" + output);

				output = output/12;
				
				synchronized(arm)
				{
					if(enableOutput)
					{
						//					System.out.print("t" + targetAngle + " a" + getShoulderAngle() + "  e" + error + "  s"+ sumError + "  o" + output);
						shoulderMotor.set(output);
						//					System.out.println(" >>> ");
					}
					//				System.out.println("");
					try {
						//					Thread.sleep(0, 500000);  // 500 000 Nanoseconds is 0.5 milliseconds
						Thread.sleep(20, 000000);  // 500 000 Nanoseconds is 0.5 milliseconds
					} catch (InterruptedException e) {
						// DON'T CARE, JUST WANT TO WAIT...
					}
				}
			}
			////////////////////////////////////////////////////////////////////////////////////////
			/// END TASK LOOP //////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////
		}

		public void enableControlOutput()
		{
			synchronized(arm)
			{
				enableOutput = true;
			}
		}
		public void disableControlOutput()
		{
			synchronized(arm)
			{
				enableOutput = false;
			}
		}
    }
}
