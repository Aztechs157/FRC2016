package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GrabBoulderAuto extends Command {

	// this command is designed to be executed "whileHeld()"
	
	// At start of command
	// move bar to pregrab position
	// spin up shooter and grabber
	
	// wait for IR sensor to see ball in range

	// when ball is in range
	// move bar to grab position

	// wait for IR sensor to see ball in holder

	// stop command

	// when command stops	
	// stop shooter and grabber, leave arm in pregrab position
	private enum CommandState
	{
		WAIT_FOR_IN_RANGE,
		WAIT_FOR_IN_HOLDER,
		BALL_IN_HOLDER;
	}

	CommandState commandState;
	boolean commandComplete;
	
    public GrabBoulderAuto() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
       	Robot.arm.shoulderGotoPosition(Position.PREPARE_FOR_BOULDER);
    	Robot.ballHandler.startIntake();
    	System.out.println("AUTOMATIC GRAB BOULDER - move arm to pregrab position, start intake motors");
    	commandState = CommandState.WAIT_FOR_IN_RANGE;
    	commandComplete = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	switch(commandState)
    	{
    	case WAIT_FOR_IN_RANGE:
    	{
    		// while we are waiting for the ball to be in range, if we notice it is in grabbable range
    		if(Robot.ballHandler.isBallGrabable() == true)
    		{
    			// Grab the ball
    	    	Robot.arm.shoulderGotoPosition(Position.GRAB_BOULDER);
    	    	System.out.println("AUTOMATIC GRAB BOULDER - in range ---------- GRAB");
    	    	commandState = CommandState.WAIT_FOR_IN_HOLDER;
    		}
    	}
    	break;
    	case WAIT_FOR_IN_HOLDER:
    	{
      		// while we are waiting for the ball to get in the hopper, if we notice it is in the hopper
    		if(Robot.ballHandler.isBallInHolder() == true)
    		{
    	    	System.out.println("AUTOMATIC GRAB BOULDER - in hopper ----------STOPPING");
    	       	Robot.arm.shoulderGotoPosition(Position.PREPARE_FOR_BOULDER);
    	    	Robot.ballHandler.stopAll();
    	    	commandState = CommandState.BALL_IN_HOLDER;
    		}   		
    	}
    	break;
    	case BALL_IN_HOLDER:
    	{
    		// do nothing until the user releases the button
    	}
    	break;
    	default:
    		// should never get here so BAIL
    		commandComplete = true;
    		break;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return commandComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
       	Robot.arm.shoulderGotoPosition(Position.PREPARE_FOR_BOULDER);
    	Robot.ballHandler.stopAll();
    	System.out.println("AUTOMATIC GRAB BOULDER END - move arm to pregrab position, stop intake motors");
   }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
       	Robot.arm.shoulderGotoPosition(Position.PREPARE_FOR_BOULDER);
    	Robot.ballHandler.stopAll();
    	System.out.println("AUTOMATIC GRAB BOULDER INTERRUPT- move arm to pregrab position, stop intake motors");
    }
}
