// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2016.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc157.FRC2016.Robot;

/**
 *
 */
public class TeleopDrive extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public TeleopDrive() {
    	requires(Robot.drive);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	// Set Control Values
    	double left;
    	double right;
    	
    	// Arcade (arcade on one stick with stick Z for rotate in place (twist))
    	double arcX = 0;//Robot.oi.operator.getX();   // left/right stick
    	double arcY = 0;//Robot.oi.operator.getY();   // forward/backward stick
    	double arcRot = 0;//Robot.oi.operator.getZ(); // twist stick
    	
    	left = arcY - arcX - arcRot/2;
    	right = arcY + arcX + arcRot/2;
    	   	
    	// Tank  (Tank on two sticks)
      	left += Robot.oi.driverLeft.getY();      // forward/backward left stick
    	right += Robot.oi.driverRight.getY();    // forward/backward right stick
    	
    	// Logitech  (Arcade mode on left stick)
    	//TEMP: Disable Logitech
    	double logX = 0; //Robot.oi.logitechDriver.getLeftX();  // forward/backward left stick
    	double logY = 0; //Robot.oi.logitechDriver.getLeftY();  // left/right left stick
    	
    	double tempLeft = logX + logY;
    	double tempRight = logX - logY;
    	
    	left += tempLeft;
    	right += tempRight;
    	    	
    	// DEMO MODE DRIVE REDUCTION
//    	double maxPercent = 0.33;
    	double maxPercent = 1.00;
    	
    	// Feed Control Values to Drive
    	Robot.drive.setLeftDrive(left * maxPercent);
    	Robot.drive.setRightDrive(right * maxPercent);
    	    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
