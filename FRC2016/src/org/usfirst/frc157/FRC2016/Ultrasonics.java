package org.usfirst.frc157.FRC2016;

import java.util.concurrent.atomic.AtomicLong;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Ultrasonics {

	private final static float UltrasonicKickoffPulseLenght = (float) 0.00001; // seconds 
//	private final static float UltrasonicKickoffPulseLenght = (float) 0.001; // seconds 
	
	public enum UltrasonicSensor
	{
		// Name       Index, SPI Byte,   V/in,   min, max    Min, Max ensor range are in inches

		FRONT_LEFT    (0,    (byte)0x08, 0.0098, 6.0, 254.0),
		FRONT_RIGHT   (1,    (byte)0x09, 0.0098, 6.0, 254.0),
		RIGHT_FRONT   (2,    (byte)0x0A, 0.0098, 6.0, 254.0),
		RIGHT_REAR    (3,    (byte)0x0B, 0.0098, 6.0, 254.0),
		REAR_RIGHT    (4,    (byte)0x0C, 0.0098, 6.0, 254.0),
		REAR_LEFT     (5,    (byte)0x0D, 0.0098, 6.0, 254.0),
		LEFT_REAR     (6,    (byte)0x0E, 0.0098, 6.0, 254.0),
		LEFT_FRONT    (7,    (byte)0x0F, 0.0098, 6.0, 254.0);

		public final int    index;            // Ordinal index (for storing value in array)
		private final byte   spiByte;          // Byte to write to SPI to setup mux to read this sensor (mux uses low nibble)
		private final double voltsPerInch;     // volts/inch sensor calibration data
		private final double minRangeInches;   // minimum range for sensor
		private final double maxRangeInches;   // maximum range for sensor

		
		UltrasonicSensor(int index, byte spiByte, double voltsPerInch, double minRangeInches, double maxRangeInches) {
			this.index = index;
			this.spiByte = spiByte;
			this.voltsPerInch = voltsPerInch;
			this.minRangeInches = minRangeInches;
			this.maxRangeInches = maxRangeInches;
		}
		
		// set the numnber of sensors defined in the enum automatically
		private static final int NumSensors = UltrasonicSensor.values().length;
	}

	private static final int MAX_ULTRASONIC_RANGE_TIME = 50; // milliseconds (from Maxbotics LV-MaxSonar EZ Series datasheet)
	private static final int ULTRASONIC_LOOP_TIME = MAX_ULTRASONIC_RANGE_TIME * UltrasonicSensor.NumSensors;  // milliseconds
	
	// ranges is where the ultrasonic task stores the range data readings (in unprocessed format)
	//  volatile since the task changes it asynchronously to the rest of the bot
	//  accesses should be protected with "synchronized(ranges)" to prevent conflicting accesses 
	public class Reading
	{
		private double value;
		private double time;
		
		Reading(double value, double time)
		{
			this.value = value;
			this.time = time;
		}

		public double getValue() {
			return value;
		}

		public double getTime() {
			return time;
		}		
		
	}
    volatile Reading reading[] = new Reading[8];
    
	private AnalogInput muxedUltrasonicInput;
	private DigitalOutput ultrasonicKickstartLine;
	private SPI muxSPI;

	public Ultrasonics(int analogInput, int kickstartLineDigtalOutput, Port spiPort)
	{
		System.out.println("Initializing the Ultrasonics");
		
		for(int idx=0; idx<8; idx++)
		{
			synchronized(reading)
			{
				reading[idx] = new Reading(-1.1, 0.0);
			}
		}
		
		// Create the analog input for the Analog Mux output
		if(muxedUltrasonicInput == null)
		{
			muxedUltrasonicInput = new AnalogInput(analogInput);
			muxedUltrasonicInput.setAverageBits(1);
			muxedUltrasonicInput.setOversampleBits(1);
		}
		
		// Create the DIO for the Kisckstart
		if(ultrasonicKickstartLine == null)
		{
			ultrasonicKickstartLine = new DigitalOutput(kickstartLineDigtalOutput);
    		System.out.println("Ultrasonics Kickstart Line Ready");
		}
		
		// Create the SPI Port
		if(muxSPI == null)
		{
			muxSPI = new SPI(spiPort);
    		System.out.println("SPI Port Created");
		}
				
		// Configure the SPI to control the Analog Mux
		if(muxSPI != null)
		{
			muxSPI.setClockRate(1000000);
			muxSPI.setChipSelectActiveHigh();
			muxSPI.setClockActiveHigh();
			muxSPI.setLSBFirst();
			muxSPI.setSampleDataOnRising();
			System.out.println("SPI Port Configured");
		}
		
		// start the thread that will just keep reading the ultrasonics
		if(ultrasonicTask == null)
		{
			ultrasonicTask = new UltrasonicTask(this);
			ultrasonicTask.setDaemon(true);
			ultrasonicTask.start();
		}
	}
	
	public Reading getSensorReadingInVolts(UltrasonicSensor sensor)
	{
		double sensorVoltage = -10;
		double timeOfReading = 0;
		
		synchronized(reading)
		{
			sensorVoltage = reading[sensor.index].value;
			timeOfReading = reading[sensor.index].time;
		}

		Reading result = new Reading(sensorVoltage, timeOfReading);
		return result;
	}
	
	public Reading getSensorReadingInInches(UltrasonicSensor sensor)
	{
		double sensorVoltage = -10;
		double timeOfReading = 0;
		
		synchronized(reading)
		{
			sensorVoltage = reading[sensor.index].value;
			timeOfReading = reading[sensor.index].time;
		}

		// Convert voltage to inches (and limit it to sensor range capability)
		double distanceInInches = sensorVoltage / sensor.voltsPerInch;
		distanceInInches = (distanceInInches > sensor.maxRangeInches) ? sensor.maxRangeInches : distanceInInches;
		distanceInInches = (distanceInInches < sensor.minRangeInches) ? sensor.minRangeInches : distanceInInches;  

		Reading result = new Reading(distanceInInches, timeOfReading);
		return result;
	}

	public long getUltrasonicLoopCount()
	{
		return ultrasonicTask.getLoopCount();
	}
	
	// Class for the thread that cycles through the ultrasonic sensors 
	//   setting the analog mux address and reading the associated sensor data
	private static class UltrasonicTask extends Thread {
		
		private static int instanceCount = 0;
		private AtomicLong loopCount = new AtomicLong(0);
		private static final double ULTRASONIC_STABILIZED_READ_TOLERANCE = 0.1;
		private boolean stop = false;
		Ultrasonics ultrasonics;

		private class UltrasonicTaskSensorData
		{
			byte spiByte;
			int index;
			UltrasonicTaskSensorData(int index, byte spiByte)
			{
				this.spiByte = spiByte;
				this.index = index;
			}
		}
		UltrasonicTaskSensorData ultrasonicTaskSensorData[] = {
				new UltrasonicTaskSensorData(UltrasonicSensor.FRONT_LEFT.index,  UltrasonicSensor.FRONT_LEFT.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.FRONT_RIGHT.index, UltrasonicSensor.FRONT_RIGHT.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.RIGHT_FRONT.index, UltrasonicSensor.RIGHT_FRONT.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.RIGHT_REAR.index,  UltrasonicSensor.RIGHT_REAR.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.REAR_RIGHT.index,  UltrasonicSensor.REAR_RIGHT.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.REAR_LEFT.index,   UltrasonicSensor.REAR_LEFT.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.LEFT_REAR.index,   UltrasonicSensor.LEFT_REAR.spiByte),
				new UltrasonicTaskSensorData(UltrasonicSensor.LEFT_FRONT.index,  UltrasonicSensor.LEFT_FRONT.spiByte)
		};

		public UltrasonicTask(Ultrasonics parent) {
			instanceCount ++;
			if(instanceCount > 1)
			{
				System.out.println("==== ERROR - Multiple (" +  instanceCount + ") Ultrasonic Task Instances =====================");
			}
			stop = false;
			ultrasonics = parent;
			System.out.println("Ultrasonic Task Created");			
		}
		
		public long getLoopCount()
		{
			long count = loopCount.get();
			return count;
		}

		@Override
		public void run() {
			System.out.println("Ultrasonic Task Running");
			long loopNum;
			
			double sensorVoltage, sensorVoltageOldRead; // sensor reads - must approximately match when read to be considered valid
			byte spiCommand[] = {(byte)0x00};

			// pulse the DIO to kickstart the ultrasonics into automatically reading ranges
			ultrasonics.ultrasonicKickstartLine.set(false);

			ultrasonics.ultrasonicKickstartLine.pulse(RobotMap.NavUltrasonicKickstartLineDigitalOut, UltrasonicKickoffPulseLenght);

			// wait for all the the ultrasonics to range at least once before reading them
			double startTime = Timer.getFPGATimestamp();  // seconds
			long waitTime = ULTRASONIC_LOOP_TIME;         // milliseconds
			System.out.println("Waiting for initial sensor acqisitions");
				try {
					Thread.sleep(waitTime);     // milliseconds
				} catch (InterruptedException e) {
					// if interrupted the next waitTime is the loop time less the time we already waited
					waitTime = ULTRASONIC_LOOP_TIME - (long)((Timer.getFPGATimestamp() - startTime) * 1000.0);
				}
			System.out.println("Beginning sensor task looping");
			////////////////////////////////////////////////////////////////////////////////////////
			/// TASK LOOP //////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////
			while(!stop)
			{
				for(int idx = 0; idx < 8; idx++)
				{
//					System.out.print(ultrasonicTaskSensorData[idx].index);
					// Write the LT1390 addressing nibble (4 bits) via the SPI bus
					spiCommand[0] = ultrasonicTaskSensorData[idx].spiByte;	
					ultrasonics.muxSPI.write(spiCommand, 1); 

					// wait for the reading to stabilize after the mux switches
					// mux gets there in 400ns or less, it's less clear how long the roborio takes to get a stable reading
					try {
						Thread.sleep(1);     // milliseconds
//						Thread.sleep(0, 500000);  // .5ms is 500000 ns
					} catch (InterruptedException e) {
						// DO NOTHING
					}
					
					// read the analog input (two in a row must match to avoid reading on an update)
					int readCount = 0;
					sensorVoltage = ultrasonics.muxedUltrasonicInput.getVoltage();
					do
					{
						readCount++;
						sensorVoltageOldRead = sensorVoltage;
						sensorVoltage = ultrasonics.muxedUltrasonicInput.getVoltage();
					}  while ((Math.abs(sensorVoltage - sensorVoltageOldRead) > ULTRASONIC_STABILIZED_READ_TOLERANCE) &&
							(readCount < 10));
//					if(readCount >= 10)
//					{
//						System.out.println("------- Ultrasonic did not settle  - idx[" + idx + "]#" + ultrasonicTaskSensorData[idx].index);
//					}
//					else
//					{
//						System.out.println("+++++++ Ultrasonic settled         - idx[" + idx + "]#" + ultrasonicTaskSensorData[idx].index);						
//					}

			        // store the sensor value for use and note the time
					synchronized(ultrasonics.reading)
					{						
						ultrasonics.reading[ultrasonicTaskSensorData[idx].index].time = Timer.getFPGATimestamp();
						ultrasonics.reading[ultrasonicTaskSensorData[idx].index].value = sensorVoltage;
					}
 				}
				loopNum = loopCount.incrementAndGet();
		    	SmartDashboard.putInt("Ultrasonic Loop Count", (int)loopNum);
//				System.out.println(".");
			}
			////////////////////////////////////////////////////////////////////////////////////////
			/// END TASK LOOP //////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////
			System.out.println("======= ERROR ULTRASONIC TASK ENDED =================");
		}

	}
	private UltrasonicTask ultrasonicTask;

}


