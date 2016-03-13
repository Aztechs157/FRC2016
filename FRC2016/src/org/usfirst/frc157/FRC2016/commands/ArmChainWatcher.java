package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.subsystems.Arm;

import com.ni.vision.NIVision.CurvatureAnalysisReport;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmChainWatcher extends Command {
    
    /////////////////////////////////////////////////////////////////////////
    ///
    ///      * * U P D A T E * *
    ///
    /// Polarity of Retract and Extend has been fixed.
    /// 
    ///////////////////////////////////////////////////////////////////////// 

	private int currentBurpZone = 0;
	
	// ZONE 1 is between zoneAngles[0] and zoneAngles[1]
    // ZONE 2 is between zoneAngles[1] and zoneAngles[2]
	// etc . . .
	// CAUTION: length of a zoneAngles[] must be (ZONE_COUNT + 1) 
	private final double[] zoneAngles = {-9999.0, 15.0, 30.0, 60.0, 9999.0};
	private final int ZONE_COUNT = 4;
    private final double zoneAngleThreshold = 2.5;  //hysteresis between zones
	
    public ArmChainWatcher( ) {
        ///
        /// Does NOT require Robot.arm to prevent other commands from stopping
        ///
        //requires(Robot.arm);
    	
        System.out.println("Creating Arm Chain Watcher");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        currentBurpZone = getCurrentArmZone();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
           	
        double currentAngle = Robot.arm.getShoulderAngle();
        
        // range check the currentZone because it is used as array index
        if (currentBurpZone < 0) currentBurpZone = 0;
        if (currentBurpZone > ZONE_COUNT) currentBurpZone = ZONE_COUNT;

        //Are we outside our current zone by more than zoneAngleThreshold?
        int newBurpZone = currentBurpZone;

        if (currentAngle < zoneAngles[currentBurpZone] - zoneAngleThreshold)
        {
            newBurpZone = currentBurpZone - 1 ;
            if (newBurpZone < 0) newBurpZone = 0;
            System.out.println("New Arm Zone (-): " + newBurpZone);
        }
        else if (currentAngle > zoneAngles[currentBurpZone+1] + zoneAngleThreshold)
        {
            newBurpZone = currentBurpZone + 1;
            if (newBurpZone > ZONE_COUNT) newBurpZone = ZONE_COUNT;
            System.out.println("New Arm Zone (+): " + newBurpZone);
        }
        

        if (newBurpZone < currentBurpZone)
        {
            new ArmChainBurp(false).start();
        }
        else if (newBurpZone > currentBurpZone)
        {
            new ArmChainBurp(true).start();
        }
        
        currentBurpZone = newBurpZone;
    }
            

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //run forever
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        
    }
    
    private int getCurrentArmZone()
    {
        int zone = 0;
        double currentAngle = Robot.arm.getShoulderAngle();
        for(double angle : zoneAngles)
        {
            if (currentAngle > angle)
            {
                zone++;
            }
        }
        return zone;
    }
    
}
