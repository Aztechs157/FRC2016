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

import com.ni.vision.NIVision.CalibrationThumbnailType;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class Drive extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	public class Distance
	{
		public double left;
		public double right;
		public double combined;
	}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    private final RobotDrive roboDrive = RobotMap.roboDrive;
    public void drive(GenericHID stick)
    {
        roboDrive.arcadeDrive(stick, true);
    }
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final CANTalon leftDriveA = RobotMap.driveLeftDriveA;
    private final CANTalon leftDriveB = RobotMap.driveLeftDriveB;
    private final CANTalon rightDriveA = RobotMap.driveRightDriveA;
    private final CANTalon rightDriveB = RobotMap.driveRightDriveB;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        setDefaultCommand(new TeleopDrive());
   }
    
    public Drive()
    {
    	// Set Talon Modes
    	configureControllers();
    	zeroDistance();
    }
    
    public void setLeftDrive(double drive)
    {
    	leftDriveA.set(-drive);
    	SmartDashboard.putDouble("LeftDrive", drive);
    }

    public void setRightDrive(double drive)
    {
    	rightDriveA.set(drive);    	
    	SmartDashboard.putDouble("RightDrive", drive);
    }

    public void setBrakeModeOn(boolean setBrakeOn)
    {
    		rightDriveA.enableBrakeMode(setBrakeOn);
    }
    
    private void configureControllers()
    {
    	leftDriveA.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	rightDriveA.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

    	// Set the followers
    	leftDriveB.changeControlMode(CANTalon.TalonControlMode.Follower);
    	leftDriveB.set(leftDriveA.getDeviceID());
    	//leftDriveB.reverseOutput(false);
    	rightDriveB.changeControlMode(CANTalon.TalonControlMode.Follower);
    	rightDriveB.set(rightDriveA.getDeviceID());
    	//rightDriveB.reverseOutput(false);
    	
    	// TODO consider setting ramp rate
    }
    
    private int leftStartCount;
    private int rightStartCount;
    private static final double TRANSMISSION_GEAR_RATIO = 14.88;  // 14.88 : 1  // don't care encoder is on drive shaft
    private static final double WHEEL_CIRCUMFRENCE = 25.13; // inches
    private static final int ENCODER_COUNTS_PER_REV = 360; 
  
    public void zeroDistance()
    {
    	leftStartCount = leftDriveA.getEncPosition();
    	rightStartCount = rightDriveA.getEncPosition();
    }
    
    public Distance getDistance()
    {
    	Distance dist = new Distance();
    	dist.left  = WHEEL_CIRCUMFRENCE * (((double)(leftDriveA.getEncPosition() - leftStartCount))/((double)ENCODER_COUNTS_PER_REV));
    	dist.right = WHEEL_CIRCUMFRENCE * (((double)(rightDriveA.getEncPosition() - rightStartCount))/((double)ENCODER_COUNTS_PER_REV));
    	dist.combined = (dist.left + dist.right)/2;
    	return dist;
    }

    private static boolean stopAutoDrive = false;
	public void stopAuto(boolean stopAutoDrive) {
		this.stopAutoDrive = stopAutoDrive;
	}
	public boolean stopAuto()
	{
		return stopAutoDrive;
	}
}

