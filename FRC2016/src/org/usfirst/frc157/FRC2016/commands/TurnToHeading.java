package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToHeading extends Command {

	private final static double HEADING_TOLERANCE = 2.0;      // degrees
	private final static double INTEGRATION_TOLERANCE = 20.0; // degrees

	private final static double DELTA_CONSTANT = (1.0/360.0);  // P type constant for PID like control below
	private final static double INTEGRATION_CONSTANT = 0.01;    // I type constant for PID like control below
	
	private double targetHeading;
	private double sumDeltaHeading;
	
    public TurnToHeading(double heading) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drive);
    	targetHeading = heading;
    	sumDeltaHeading = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double deltaHeading = Robot.navigation.getHeading() - targetHeading;
    	
    	// bring the delta heading into -180 to 180
    	while (deltaHeading > 180)
    	{
    		deltaHeading = deltaHeading - 360;
    	}
    	while (deltaHeading < -180)
    	{
    		deltaHeading = deltaHeading + 360;
    	}
    	
    	
    	if(Math.abs(deltaHeading) <  INTEGRATION_TOLERANCE)
    	{
    		sumDeltaHeading = sumDeltaHeading + INTEGRATION_CONSTANT * deltaHeading;
    	}
    	else
    	{
    		sumDeltaHeading = 0;
    	}
    	
    	double driveValue = deltaHeading * DELTA_CONSTANT + sumDeltaHeading;

    	double left = -driveValue;
    	double right = driveValue;

    	System.out.print("D:" + deltaHeading);
    	System.out.println(" L:" + left + " R:" + right);

    	Robot.drive.setLeftDrive(left);
    	Robot.drive.setRightDrive(right);    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Math.abs((Robot.navigation.getHeading() - targetHeading)%360) < HEADING_TOLERANCE)
    	{
    		// TODO do we want to set brake mode here?
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
