package org.usfirst.frc157.FRC2016.subsystems;

import org.usfirst.frc157.FRC2016.ADIS16448_IMU;
import org.usfirst.frc157.FRC2016.UltrasonicRangeSensor;
import org.usfirst.frc157.FRC2016.ADIS16448_IMU.ReadTask;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Navigation extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private static ADIS16448_IMU imu;
	private static UltrasonicRangeSensor[] ultrasonic = new UltrasonicRangeSensor[8];
	private static double[] ultrasonicRange;
	
	private final static int FrontRight = 0;
	private final static int FrontLeft  = 1;
	
	private final static int RearRight  = 2;
	private final static int RearLeft   = 3;

	private final static int LeftFore   = 4;
	private final static int LeftAft    = 5;

	private final static int RightFore  = 6;
	private final static int RightAft   = 7;
	
	private static class UltrasonicTask implements Runnable {
		private boolean stop = false;
		public UltrasonicTask() {
			stop = false;
		}

		@Override
		public void run() {
			while(!stop)
			{
				// read the ultrasonics
				for(int sensor = FrontRight; sensor <= RightAft; sensor ++)
				{
					if(ultrasonic[sensor] != null)
					{
						//ultrasonicRange[sensor] = ultrasonic[sensor].getRange();
					}
				}
			}
		}

		public void stop()
		{
			stop = true;
		}
	}
	private Thread ultrasonicTask;

	
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
    	
    	UltrasonicRangeSensor.CalibrationData ultrasonicCalData = new UltrasonicRangeSensor.CalibrationData();
        ultrasonicCalData.voltsPerInch = 0.0098;
        ultrasonicCalData.minRangeInches = 6.0;
        ultrasonicCalData.maxRangeInches = 254;

//   TODO Fix to use ultrasonics correctly 	for(int sensor = FrontRight; sensor <= RightAft; sensor ++)
    		for(int sensor = FrontRight; sensor < RightAft; sensor ++)
    	{
    		System.out.println("Sensor " + sensor);
    		if(ultrasonic[sensor] == null)
    		{
    			ultrasonic[sensor] = new UltrasonicRangeSensor(sensor, ultrasonicCalData);
    		}
    	}
    	
    	// start ultrasonic task
    	
    	// TODO Kick off Ultrasonic Round Robin
    	ultrasonicTask = new Thread(new UltrasonicTask());
    	ultrasonicTask.setDaemon(true);
    	ultrasonicTask.start();
    	
    	System.out.println("Navigation Subsystem Initialized");
    }
}

