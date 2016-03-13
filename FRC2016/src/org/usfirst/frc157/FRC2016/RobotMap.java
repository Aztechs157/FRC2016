// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2016;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public final static int DriveLeftATalonID = 1;
	public final static int DriveLeftBTalonID = 3;
	public final static int DriveRightATalonID = 6;
	public final static int DriveRightBTalonID = 8;
	
	public final static int ArmShoulderTalonID = 2;
	public final static int ArmExtenderATalonID = 4;
	public final static int ArmExtenderBTalonID = 7;
	
	public final static int ArmHomeSwitchDigitalInID = 9;
	public final static int ArmExtendedSwitchDigitalInID = 8;
	public final static int ArmRetractedSwitchDigitalInID = 7;
	
	public final static int BallLeftShooterTalonID = 5;
	public final static int BallRIghtShooterTalonID = 9;
	public final static int BallRetireverTalonID = 10;	
	public final static int BallIRRangefinderAnalogIn = 2;
	public final static int BallInHolderSwitchDigitialInID = 0;
	
	public final static int NavUltrasonicRangefinderAnalogIn = 1;
	public final static int NavUltrasonicKickstartLineDigitalOut = 2;
	public final static Port NavUltrasonicMuxSPIPort = SPI.Port.kOnboardCS0;

	public final static int OnboardModeSelectSwitchAnalogInID = 3;
	
	public static RobotDrive roboDrive;
	public static Timer time;
	
	public final static int NavIMUInterruptDigitalIn = 0;  // DO NOT CHANGE - hard coded/hardware based in IMU class
	public final static Port NavIMUSPIPort = SPI.Port.kMXP; // DO NOT CHANGE - hard coded/hardware based in IMU class
			
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static CANTalon driveLeftDriveA;
    public static CANTalon driveRightDriveA;
    public static CANTalon driveLeftDriveB;
    public static CANTalon driveRightDriveB;
    public static CANTalon armShoulderMotor;
    public static CANTalon armExtenderMotorA;
    public static CANTalon armExtenderMotorB;
    public static CANTalon ballHandlerLeftShooter;
    public static CANTalon ballHandlerRightShooter;
    public static CANTalon ballHandlerBallRetriever;
    public static AnalogInput ballHandlerBallRangefinder;
    
    public static AnalogSelectSwitch modeSelect;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
    	modeSelect = new AnalogSelectSwitch(OnboardModeSelectSwitchAnalogInID);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveLeftDriveA = new CANTalon(DriveLeftATalonID);
        LiveWindow.addActuator("Drive", "LeftDriveA", driveLeftDriveA);
        driveLeftDriveB = new CANTalon(DriveLeftBTalonID);
        LiveWindow.addActuator("Drive", "LeftDriveB", driveLeftDriveB);
        
        driveRightDriveA = new CANTalon(DriveRightATalonID);
        LiveWindow.addActuator("Drive", "RightDriveA", driveRightDriveA);         
        driveRightDriveB = new CANTalon(DriveRightBTalonID);
        LiveWindow.addActuator("Drive", "RightDriveB", driveRightDriveB);

        armShoulderMotor = new CANTalon(ArmShoulderTalonID);
        LiveWindow.addActuator("Arm", "ShoulderMotor", armShoulderMotor);

        armExtenderMotorA = new CANTalon(ArmExtenderATalonID);
        LiveWindow.addActuator("Arm", "ExtenderMotorA", armShoulderMotor);
        armExtenderMotorB = new CANTalon(ArmExtenderBTalonID);
        LiveWindow.addActuator("Arm", "ExtenderMotorB", armShoulderMotor);
        
        ballHandlerLeftShooter = new CANTalon(BallLeftShooterTalonID);
        LiveWindow.addActuator("BallHandler", "LeftShooter", ballHandlerLeftShooter);
        
        ballHandlerRightShooter = new CANTalon(BallRIghtShooterTalonID);
        LiveWindow.addActuator("BallHandler", "RightShooter", ballHandlerRightShooter);
        
        ballHandlerBallRetriever = new CANTalon(BallRetireverTalonID);
        LiveWindow.addActuator("BallHandler", "BallRetriever", ballHandlerBallRetriever);
                
        System.out.println("Robot INIT COMPLETE");
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
