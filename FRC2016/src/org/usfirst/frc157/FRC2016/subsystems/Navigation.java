package org.usfirst.frc157.FRC2016.subsystems;

import org.usfirst.frc157.FRC2016.ADIS16448_IMU;
import org.usfirst.frc157.FRC2016.Ultrasonics;
import org.usfirst.frc157.FRC2016.ADIS16448_IMU.ReadTask;
import org.usfirst.frc157.FRC2016.RobotMap;

import edu.wpi.first.wpilibj.Ultrasonic;
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
    
    public class Position
    {
    	//       (0,0)          (maxX, 0)
    	//             +======+
    	//             #      |
    	//             #      |
    	//             #      |
    	//             +------+
    	//    (0, maxY)         (maxX, maxY)
    	//
    	public double y;  // distance in inches from far end of field '=' in above diagram  (+ down)
    	public double x;  // distance from wall'#' in above diagram (+ to the right)
    }
    
    Position robotLocation;
    
    public Navigation()
    {
     	
    	if(imu == null)
    	{	
    		imu = new ADIS16448_IMU();
    		imu.calibrate();   		
    		resetZeroHeading();
    		System.out.println("IMU Initialized");
    	}
    	
    	if(ultrasonics == null)
    	{
    		ultrasonics = new Ultrasonics(RobotMap.NavUltrasonicRangefinderAnalogIn, RobotMap.NavUltrasonicKickstartLineDigitalOut, RobotMap.NavUltrasonicMuxSPIPort);
    		System.out.println("Ultrasonics Initialized");
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
    
    public double getUltrasonicRange(Ultrasonics.UltrasonicSensor sensor)
    {
    	return ultrasonics.getSensorReadingInInches(sensor).getValue();
    }
    public double getUltrasonicVoltage(Ultrasonics.UltrasonicSensor sensor)
    {
    	return ultrasonics.getSensorReadingInVolts(sensor).getValue();
    }
   
    public enum Side
    {
    	FRONT,
    	RIGHT,
    	REAR,
    	LEFT;
    }
    
    public double ultrasonicHeading(Side side)
    {
    	double rangeA = 0;
    	double rangeB = 0;
    	// get the two ultrasonics for the side
    	switch(side)
    	{
    		case FRONT:
    			rangeA = getUltrasonicRange(Ultrasonics.UltrasonicSensor.FRONT_LEFT);
    			rangeB = getUltrasonicRange(Ultrasonics.UltrasonicSensor.FRONT_RIGHT);
    			break;
    		case RIGHT:
    			rangeA = getUltrasonicRange(Ultrasonics.UltrasonicSensor.RIGHT_FRONT);
    			rangeB = getUltrasonicRange(Ultrasonics.UltrasonicSensor.RIGHT_REAR);
    			break;
    		case REAR:
    			rangeA = getUltrasonicRange(Ultrasonics.UltrasonicSensor.REAR_LEFT);
    			rangeB = getUltrasonicRange(Ultrasonics.UltrasonicSensor.REAR_RIGHT);
    			break;
    		case LEFT:
    			rangeA = getUltrasonicRange(Ultrasonics.UltrasonicSensor.LEFT_FRONT);
    			rangeB = getUltrasonicRange(Ultrasonics.UltrasonicSensor.LEFT_REAR);
    			break;
    	};
    	
    	double heading = 0;
    	
    	return heading;
    }
    
      
    public class Location
    {
    	double x;
    	double y;
    	Location()
    	{
    		x=0; y=0;
    	}
    }
    public class NavData
    {
    	double distance;
    	double heading;
    	NavData()
    	{
    		distance = 0; heading = 0;
    	}
    }
    public NavData navigationToPoint(Location start, Location end)
    {
    	NavData navData = new NavData();
    	navData.distance = Math.sqrt(Math.abs((start.x-end.x)*(start.x-end.x) + (start.y - end.y)*(start.y - end.y)));
    	navData.heading = Math.atan2(start.y-end.y, start.x-end.x);
    	return navData;
    }

	public long getUltrasonicLoopCount() {
		return ultrasonics.getUltrasonicLoopCount();
	}
	
	public Position getPosition()
	{
		Position here;
		here = robotLocation;
		return here;
	}
	
	public void setPosition(Position here)
	{
		robotLocation = here;
	}
}

