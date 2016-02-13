package org.usfirst.frc157.FRC2016.subsystems;

import org.usfirst.frc157.FRC2016.ADIS16448_IMU;
import org.usfirst.frc157.FRC2016.Ultrasonics;
import org.usfirst.frc157.FRC2016.ADIS16448_IMU.ReadTask;
import org.usfirst.frc157.FRC2016.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Navigation extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static ADIS16448_IMU imu;
	private static Ultrasonics ultrasonics;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Navigation()
    {
     	
    	if(imu == null)
    	{	
    		imu = new ADIS16448_IMU();
    	}
    	
    	if(ultrasonics == null)
    	{
    		ultrasonics = new Ultrasonics(RobotMap.NavUltrasonicRangefinderAnalogIn, RobotMap.NavUltrasonicKickstartLineDigitalOut, RobotMap.NavUltrasonicMuxSPIPort);
    	}
   	
    	System.out.println("Navigation Subsystem Initialized");
    }
	
	//   TODO provide a nav subsystem API
    double getPitch()
    {
    	return 7;//imu.get();
    }
}

