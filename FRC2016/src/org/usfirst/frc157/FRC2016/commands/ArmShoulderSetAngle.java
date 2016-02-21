package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmShoulderSetAngle extends Command {

	private double angle;
	
    public ArmShoulderSetAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.arm.shoulderGotoAngle(angle);
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
