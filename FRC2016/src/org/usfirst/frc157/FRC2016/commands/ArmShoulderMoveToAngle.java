package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Moves arm to target angle and waits until that position is reached.
 */
public class ArmShoulderMoveToAngle extends Command {

    public static final double AT_TARGET_RANGE = 3.0; //degrees
    private double startTime;
	private double targetAngle;
	
	private static double COMMNAD_TIMEOUT = 3.0; // seconds
	
    public ArmShoulderMoveToAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
    	this.targetAngle = angle;
    	System.out.println("ArmShoulderMoveToAngle(" + angle + ")");
   }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.arm.shoulderGotoAngle(targetAngle);
    	System.out.println("ArmShoulderMoveToAngle.initialize()");
    	
    	startTime = Timer.getFPGATimestamp();
   }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if((Timer.getFPGATimestamp() - startTime) > COMMNAD_TIMEOUT)
    	{
    		System.out.println("ArmShoulderMoveToAngle.isFinished(); - COMMAND_TIMEOUT");
    		return true;
    	}
        double error = targetAngle - Robot.arm.getShoulderAngle();
        return Math.abs(error) <= AT_TARGET_RANGE; 
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.arm.shoulderGotoAngle(Robot.arm.getShoulderAngle());
        System.out.println("Move Arm Shoulder - e stop");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.arm.shoulderGotoAngle(Robot.arm.getShoulderAngle());
        System.out.println("Move Arm Shoulder - i stop");
    }
}
