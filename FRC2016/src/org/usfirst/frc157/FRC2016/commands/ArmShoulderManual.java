package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmShoulderManual extends Command {

	// Command is designed to be run "whileHeld"
	
	public enum Direction
	{
		UP,
		DOWN;
	}
	
	private Direction direction;
	
    public ArmShoulderManual(Direction direction) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.direction = direction;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(direction == Direction.UP)
    	{
    		Robot.arm.shoulderGotoPosition(Position.FULL_UP);
    		System.out.println("Set Arm Shoulder - FULL UP");
    	}
    	else if(direction == Direction.DOWN)
    	{
    		Robot.arm.shoulderGotoPosition(Position.FULL_DOWN);
    		System.out.println("Set Arm Shoulder - FULL DOWN");
    	}
    		
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
    	Robot.arm.shoulderGotoAngle(Robot.arm.getShoulderAngle());
    	System.out.println("Set Arm Shoulder - e stop");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.arm.shoulderGotoAngle(Robot.arm.getShoulderAngle());
    	System.out.println("Set Arm Shoulder - i stop");
    }
}
