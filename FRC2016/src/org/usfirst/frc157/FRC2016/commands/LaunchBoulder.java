package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LaunchBoulder extends Command {

	private double startTime;
	private final static double SHOT_DURATION = 0.5; // seconds
	
    public LaunchBoulder() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTime = Timer.getFPGATimestamp();
    	Robot.ballHandler.startShoot();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// Once the shot duration time is done, stop
    	if(Timer.getFPGATimestamp() > (startTime + SHOT_DURATION))
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
    	Robot.ballHandler.stopAll();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.ballHandler.stopAll();
    }
}
