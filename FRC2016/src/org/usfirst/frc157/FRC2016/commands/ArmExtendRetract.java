package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmExtendRetract extends Command {

	private boolean extend;
	private boolean finished;
	
    public ArmExtendRetract(boolean extendRequest) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
    	extend = extendRequest;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(extend && Robot.arm.extendAllowedByTime())
    	{
    		finished = Robot.arm.armExtend();
    	}
    	else  // can always retract
    	{
    		finished = Robot.arm.armRetract();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
