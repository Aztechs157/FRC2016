package org.usfirst.frc157.FRC2016.commands;

import org.usfirst.frc157.FRC2016.Robot;
import org.usfirst.frc157.FRC2016.RobotMap;
import org.usfirst.frc157.FRC2016.Ultrasonics.UltrasonicSensor;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PrintIRRange extends Command {

    public PrintIRRange() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	System.out.println("PrintIRRange.Initialize()");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	System.out.print("Range = " + Robot.ballHandler.getBoulderRange()/2.54);
//    	System.out.print(RobotMap.modeSelect.getPosition().postionName);
//    	System.out.print(" E-" + Robot.arm.getArmExtendedSwitch());
//    	System.out.print(" R-" + Robot.arm.getArmRetractedSwitch());
//    	System.out.print(" S-" + Robot.arm.getShoulderHomeSwitch());

    	
    	System.out.println("<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>");
    	System.out.println("   -- Arm Angle: " + Robot.arm.getShoulderAngle());    	
    	System.out.println("<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>");
    	System.out.println("FRONT_LEFT   " + UltrasonicSensor.FRONT_LEFT.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.FRONT_LEFT));
    	System.out.println("FRONT_RIGHT: " + UltrasonicSensor.FRONT_RIGHT.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.FRONT_RIGHT));
    	System.out.println("RIGHT_FRONT: " + UltrasonicSensor.RIGHT_FRONT.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.RIGHT_FRONT));
    	System.out.println("RIGHT_REAR:  " + UltrasonicSensor.RIGHT_REAR.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.RIGHT_REAR));
    	System.out.println("REAR_RIGHT:  " + UltrasonicSensor.REAR_RIGHT.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.REAR_RIGHT));
    	System.out.println("REAR_LEFT:   " + UltrasonicSensor.REAR_LEFT.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.REAR_LEFT));
    	System.out.println("LEFT_REAR:   " + UltrasonicSensor.LEFT_REAR.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.LEFT_REAR));
    	System.out.println("LEFT_FRONT:  " + UltrasonicSensor.LEFT_FRONT.index + ":  " + Robot.navigation.getUltrasonicRange(UltrasonicSensor.LEFT_FRONT));
    	System.out.println(">Loops - " + Robot.navigation.getUltrasonicLoopCount() + " <");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("PrintIRRange.end()");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
