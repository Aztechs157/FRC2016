package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveMoveDistance extends Command {

	private double leftSpeed;
	private double rightSpeed;
	private double distance;
	
	private boolean reachedDestination;
	
	// Note: be smart about setting the speeds and distance, it is possible to set
	//   the arguments to this funciton such that it will never reach the disatance
	//   e.g. spin in place will never get to the distance
    public AutoDriveMoveDistance(double leftSpeed, double rightSpeed, double distance) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive);
    	this.leftSpeed = -leftSpeed;
    	this.rightSpeed = -rightSpeed;
    	this.distance = Math.abs(distance);
    	System.out.println("AutoDriveMoveDistance(" + leftSpeed + ", " + rightSpeed + ", " + distance +")");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.zeroDistance();
    	Robot.drive.setBrakeModeOn(true);
    	Robot.drive.setLeftDrive(leftSpeed);
    	Robot.drive.setRightDrive(rightSpeed); 
    	reachedDestination = false;    	
    	Robot.drive.stopAuto(false);
    	System.out.println("AutoDriveMoveDistance.initialize()");
   }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Math.abs(Robot.drive.getDistance().combined) > distance)
    	{
    		reachedDestination = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// if auto drive has been overridden, this command completes
    	if(Robot.drive.stopAuto())
    	{
    		return true;
    	}
    	
        return (reachedDestination);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setLeftDrive(0.0);
    	Robot.drive.setLeftDrive(0.0);    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.setLeftDrive(0.0);
    	Robot.drive.setLeftDrive(0.0);    	
    }
}
