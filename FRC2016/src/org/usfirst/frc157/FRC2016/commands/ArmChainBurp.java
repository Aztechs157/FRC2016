package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmChainBurp extends Command {
    
    /////////////////////////////////////////////////////////////////////////
    ///
    ///      * * U P D A T E * *
    ///
    /// Polarity of Retract and Extend has been fixed.
    /// 
    ///////////////////////////////////////////////////////////////////////// 

	private boolean retract;
	
    public ArmChainBurp(boolean retractRequest) {
      
        ///
        /// Does NOT require Robot.arm to prevent other commands from stopping
        ///
        //requires(Robot.arm);
    	retract = retractRequest;
    	System.out.println("Creating arm Burp " + retract);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        
        setTimeout(Arm.CHAIN_BURP_TIME); //seconds
    	
        System.out.println("Starting chain burp " + retract);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
           	
    	if(retract)
    	{
    		Robot.arm.armRetract(Arm.CHAIN_BURP_SPEED);
    	}
    	else  // can always retract
    	{
    		Robot.arm.armExtend(Arm.CHAIN_BURP_SPEED);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return isTimedOut();
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
