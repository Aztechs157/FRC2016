package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmExtendRetract extends Command {
    
    /////////////////////////////////////////////////////////////////////////
    ///
    ///      * * C A U T I O N  * * 
    /// The Words Extend and Retract have reversed meaning in this Class
    ///
    /////////////////////////////////////////////////////////////////////////

	private boolean extend;
	private boolean finished;
	
    public ArmExtendRetract(boolean extendRequest) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
    	extend = extendRequest;
    	System.out.println("Creating arm Extend/Retract " + extend);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting arm Extend/Retract " + extend);
    	finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
        /////////////////////////////////////////////////////////////////////////
        ///
        ///      * * C A U T I O N  * * 
        /// The Words Extend and Retract have reversed meaning in this Class
        ///
        /////////////////////////////////////////////////////////////////////////
    	
    	if(extend && Robot.arm.extendAllowedByTime())
    	{
    		Robot.arm.armExtend();
    	}
    	else  // can always retract
    	{
    		Robot.arm.armRetract();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean result = false;
    	if(extend)
    	{
    		result =  !Robot.arm.getArmExtendedSwitch();
    	}
    	else
    	{
    		result = !Robot.arm.getArmRetractedSwitch();
    	}
    	return result;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
