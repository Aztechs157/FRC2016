package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveTurnToHeading extends Command {

	private final static double HEADING_TOLERANCE = 2.0;      // degrees
	private final static double INTEGRATION_TOLERANCE = 5.0; // degrees
	private final static double DELTA_CONSTANT = .5 * (1.0/360.0);  // P type constant for PID like control below
	private final static double INTEGRATION_CONSTANT = 0.001;        // I type constant for PID like control below
	
	private double targetHeading;
	private double sumDeltaHeading;

	private double startTime;
	
	private static double COMMAND_TIMEOUT = 8.0; // seconds

    public AutoDriveTurnToHeading(double heading) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drive);
    	targetHeading = heading;
    	sumDeltaHeading = 0;
    	Robot.drive.stopAuto(false);
    	System.out.println("AutoDriveTurnToHeading(" + heading + ")");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.setBrakeModeOn(true);
    	Robot.drive.stopAuto(false);
    	System.out.println("AutoDriveTurnToHeading.initialize()");
    	startTime = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double deltaHeading = 0;//Robot.navigation.getHeading() - targetHeading;
    	
    	// bring the delta heading into -180 to 180
    	while (deltaHeading > 180)
    	{
    		deltaHeading = deltaHeading - 360;
    	}
    	while (deltaHeading <= -180)
    	{
    		deltaHeading = deltaHeading + 360;
    	}    	
    	
//    	if(Math.abs(deltaHeading) <  INTEGRATION_TOLERANCE)
//    	{
//    		sumDeltaHeading = sumDeltaHeading + INTEGRATION_CONSTANT * deltaHeading;
//    	}
//    	else
//    	{
//    		sumDeltaHeading = 0;
//    	}
//    	
    	double driveValue = deltaHeading * DELTA_CONSTANT + sumDeltaHeading;

    	double left = -driveValue;
    	double right = driveValue;

    	System.out.print("I: " + driveValue + "D:" + deltaHeading);
    	System.out.println(" L:" + left + " R:" + right);

    	Robot.drive.setLeftDrive(left);
    	Robot.drive.setRightDrive(right);    	
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

    	return true;
//    	
//    	if(Math.abs((Robot.navigation.getHeading() - targetHeading)%360) < HEADING_TOLERANCE)
//    	{
//    		return true;
//    	}
//    	else
//    	{
//    		return false;
//    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setBrakeModeOn(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.setBrakeModeOn(false);
    }
}
