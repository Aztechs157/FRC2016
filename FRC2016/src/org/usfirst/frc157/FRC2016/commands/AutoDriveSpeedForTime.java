package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveSpeedForTime extends Command {

	private double leftSpeed;
	private double rightSpeed;
	
	double driveTimeSec;
    double endTime;
    
    public AutoDriveSpeedForTime(double leftSpeed, double rightSpeed, double driveTimeSec) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive);
    
        this.leftSpeed = leftSpeed;
    	this.rightSpeed = rightSpeed;
    	this.driveTimeSec = driveTimeSec;
    	System.out.println("AutoDriveSpeedForTime(" + leftSpeed + ", " + rightSpeed  + ", " + driveTimeSec +")");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("AutoDriveSpeedForTime.initialize()");
    	Robot.drive.setLeftDrive(leftSpeed);
    	Robot.drive.setLeftDrive(rightSpeed);
        Robot.drive.stopAuto(false);

    	endTime = Timer.getFPGATimestamp() + driveTimeSec;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //Wait for time to lapse
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// if auto drive has been overridden, this command completes
    	if(Robot.drive.stopAuto())
    	{
    		System.out.println("AutoDriveSpeedForTime.isFinished() - terminated on AutoStop()");
    		return true;
    	}
    
    	return Timer.getFPGATimestamp() > endTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setLeftDrive(0.0);
    	Robot.drive.setLeftDrive(0.0);    
    	System.out.println("AutoDriveSpeedForTime.end()");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.setLeftDrive(0.0);
    	Robot.drive.setLeftDrive(0.0);    	
    	System.out.println("AutoDriveSpeedForTime.interrupted()");
    }
}
