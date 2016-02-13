package org.usfirst.frc157.FRC2016.subsystems;

import org.usfirst.frc157.FRC2016.ADIS16448_IMU;
import org.usfirst.frc157.FRC2016.Ultrasonics;
import org.usfirst.frc157.FRC2016.ADIS16448_IMU.ReadTask;
import org.usfirst.frc157.FRC2016.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Navigation extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static ADIS16448_IMU imu;
	private static Ultrasonics ultrasonics;
	
	private double initialHeading;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Navigation()
    {
     	
    	if(imu == null)
    	{	
    		imu = new ADIS16448_IMU();
    		imu.calibrate();
    		
    		resetZeroHeading();
    	}
    	
    	if(ultrasonics == null)
    	{
    		ultrasonics = new Ultrasonics(RobotMap.NavUltrasonicRangefinderAnalogIn, RobotMap.NavUltrasonicKickstartLineDigitalOut, RobotMap.NavUltrasonicMuxSPIPort);
    	}
   	
    	System.out.println("Navigation Subsystem Initialized");
    }
	
	//   TODO provide a nav subsystem API
    public double getPitch()
    {
    	return imu.getPitch();
    }
    public double getRoll()
    {
    	return imu.getRoll();
    }
    public double getYaw()
    {
    	return imu.getYaw();
    }
    public double getHeading()
    {
    	double heading = imu.getAngle() - initialHeading;
    	SmartDashboard.putDouble("Heading", heading);
    	return heading;
    }
    public double getMagHeading()
    {
    	return imu.getAngle();
    }
    public void resetZeroHeading()
    {
    	initialHeading = imu.getAngle(); 
    }
}

