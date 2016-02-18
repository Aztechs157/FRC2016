package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GrabBoulderManual extends Command {

	
	// this command is designed to be executed "whileHeld()"
	
	// At start of command
	// move bar to grab postion
	// spin up shooter and grabber
	
	// when the command is stopped
	// stop the shooter and grabber
	// move bar to pregrab position
	
	
    public GrabBoulderManual() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.arm.shoulderGotoPosition(Position.GRAB_BOULDER);
    	Robot.ballHandler.startIntake();
    	System.out.println("GRAB BOULDER - move arme down, start intake motors");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ballHandler.stopAll();
    	Robot.arm.shoulderGotoPosition(Position.PREPARE_FOR_BOULDER);
    	System.out.println("GRAB BOULDER - move arm back up, stop intake motors");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.ballHandler.stopAll();
    	Robot.arm.shoulderGotoPosition(Position.PREPARE_FOR_BOULDER);
    	System.out.println("GRAB BOULDER INTERRUPTED- move arm back up, stop intake motors");
    }
}
