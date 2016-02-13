package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToHeading extends Command {

	private final static double HEADING_TOLERANCE = 5.0;
	private double targetHeading;
	
    public TurnToHeading(double heading) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drive);
    	targetHeading = heading;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double deltaHeading = Robot.navigation.getHeading() - targetHeading;
    	
//    	deltaHeading = 360-((360+deltaHeading) %360);
    	
    	double left = -deltaHeading/360;
    	double right = deltaHeading/360;
    	
    	System.out.print("D:" + deltaHeading);
    	System.out.println(" L:" + left + " R:" + right);

    	Robot.drive.setLeftDrive(left/2);
    	Robot.drive.setRightDrive(right/2);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Math.abs((Robot.navigation.getHeading() - targetHeading)%360) < HEADING_TOLERANCE)
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
