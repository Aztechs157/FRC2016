package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmExtendRetract extends Command {
    
    /////////////////////////////////////////////////////////////////////////
    ///
    ///      * * U P D A T E * *
    ///
    /// Polarity of Retract and Extend has been fixed.
    /// 
    ///////////////////////////////////////////////////////////////////////// 

	private boolean retract;
	private boolean finished;
	
    public ArmExtendRetract(boolean retractRequest) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
    	retract = retractRequest;
    	System.out.println("Creating arm Extend/Retract " + retract);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting arm Extend/Retract " + retract);
    	finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
           	
    	if(retract && Robot.arm.extendAllowedByTime())
    	{
    		Robot.arm.armRetract(Arm.RETRACT_SPEED);
    	}
    	else  // can always retract
    	{
    		Robot.arm.armExtend(Arm.EXTEND_SPEED);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean result = false;
    	if(retract)
    	{
    		result =  !Robot.arm.getArmRetractedSwitch();
    	}
    	else
    	{
    		result = !Robot.arm.getArmExtendedSwitch();
    	}
    	return result;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.arm.armExtendStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.arm.armExtendStop();
    }
}
