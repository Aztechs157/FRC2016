package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmManualMove extends Command {

    public ArmManualMove() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.arm.disableControl();  // switch to manual voltage setting
    	Robot.arm.shoulderSetVoltage(0.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double powerLevel = -Robot.oi.getOperatorThrottle();
    	Robot.arm.shoulderSetVoltage(powerLevel);
//    	System.out.println("PL: " + powerLevel);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
       	Robot.arm.shoulderSetVoltage(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
       	Robot.arm.shoulderSetVoltage(0.0);
   }
}
