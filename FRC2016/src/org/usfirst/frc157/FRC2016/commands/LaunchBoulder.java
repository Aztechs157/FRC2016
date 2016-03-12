package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LaunchBoulder extends Command {

	private double startTime;
	private final static double DELAY_BEFORE_SHOT = 0.75; // seconds
	private final static double SHOT_DURATION = 0.5; // seconds
	private boolean shooterStarted;
	
	public LaunchBoulder() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.arm);
		requires(Robot.ballHandler);
		System.out.println("LaunchBoulder()");
	}

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.arm.shoulderGotoPosition(Position.CLEAR_FOR_SHOT);
    	startTime = Timer.getFPGATimestamp();
    	shooterStarted = false;
		System.out.println("LaunchBoulder.initialize()");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if((Timer.getFPGATimestamp() > (startTime + DELAY_BEFORE_SHOT)) && (shooterStarted == false))
    	{
    		Robot.ballHandler.startShoot();
    		shooterStarted = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// Once the shot duration time is done, stop
    	if(Timer.getFPGATimestamp() > (startTime + SHOT_DURATION + DELAY_BEFORE_SHOT))
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
