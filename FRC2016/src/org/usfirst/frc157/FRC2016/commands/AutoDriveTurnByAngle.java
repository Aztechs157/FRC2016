package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveTurnByAngle extends Command {

	// TODO measure wheel distance
	private static final double WheelsDistanceSideToSide = 24.5;  // distance between center wheel on left v right side of robot
	private static final double TurnCircumfrence = Math.PI * WheelsDistanceSideToSide;
	
	// TODO calibrate turn speed to stop at
	private static final double MINIMUM_TURN_SPEED = 0.05;
	
	
	private double turnAngle;
	private double turnDistance;
	private double turnSpeed;

	// command timeout information
	private double startTime;
	private static double COMMAND_TIMEOUT = 8.0; // seconds
	
	private static final double TURN_PROPORTIONALITY_CONSTANT = 0.5;
	/**
     * @param turnAngle degrees to turn (+ is clockwise)
     */
    public AutoDriveTurnByAngle(double turnAngle) {
        // Use requires() here to declare subsystem dependencies
       	requires(Robot.drive);
       	this.turnAngle = turnAngle;
    }    
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	turnDistance = (turnAngle/360.0) * TurnCircumfrence;
    	turnSpeed = turnDistance / TurnCircumfrence;
    	Robot.drive.zeroDistance();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// use right to determine when we are done (note for CW turn right will be negative)
    	turnSpeed = TURN_PROPORTIONALITY_CONSTANT * (turnDistance + Robot.drive.getDistance().right)/TurnCircumfrence;
    	
    	Robot.drive.setLeftDrive(turnSpeed);
    	Robot.drive.setRightDrive(-turnSpeed);    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// if auto drive has been overridden, this command completes
    	if(Robot.drive.stopAuto())
    	{
    		System.out.println("Auto Turn STOPPED");
    		return true;
    	}
    	
    	if((Timer.getFPGATimestamp() - startTime) > COMMAND_TIMEOUT)
    	{
    		System.out.println("ArmShoulderMoveToAngle.isFinished(); - COMMAND_TIMEOUT");
    		return true;
    	}

    	if(turnSpeed < MINIMUM_TURN_SPEED)
    	{
    		return true;
    	}
    	else
    	{
    		return false;
    	}	
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setLeftDrive(0);
    	Robot.drive.setRightDrive(0);    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.setLeftDrive(0);
    	Robot.drive.setRightDrive(0);    	
    }
}
