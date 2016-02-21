package org.usfirst.frc157.FRC2016;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
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

		private final int    index;            // Ordinal index (for storing value in array)
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
	}

	// ranges is where the ultrasonic task stores the range data readings (in unprocessed format)
	//  volatile since the task changes it asynchronously to the rest of the bot
	//  accesses should be protected with "synchronized(ranges)" to prevent conflicting accesses 
    volatile double reading[] = new double[8];
    

	private AnalogInput muxedUltrasonicInput;
	private DigitalOutput ultrasonicKickstartLine;
	private SPI muxSPI;

	public Ultrasonics(int analogInput, int kickstartLineDigtalOutput, Port spiPort)
	{
		for(int idx=0; idx<8; idx++)
		{
			reading[idx]=-1;
		}
		
		if(muxedUltrasonicInput == null)
		{
			muxedUltrasonicInput = new AnalogInput(analogInput);
			muxedUltrasonicInput.setAverageBits(1);
			muxedUltrasonicInput.setOversampleBits(1);
		}
		if(ultrasonicKickstartLine == null)
		{
			ultrasonicKickstartLine = new DigitalOutput(kickstartLineDigtalOutput);
		}
		if(muxSPI == null)
		{
			muxSPI = new SPI(spiPort);
		}
		

//		muxSPI.setClockRate(1000000);
//		muxSPI.setChipSelectActiveHigh();
//		muxSPI.setClockActiveHigh();
//		muxSPI.setMSBFirst();
//		muxSPI.setSampleDataOnRising();
		
		muxSPI.setClockRate(1000000);
		muxSPI.setChipSelectActiveHigh();
		muxSPI.setClockActiveHigh();
		muxSPI.setLSBFirst();
		muxSPI.setSampleDataOnRising();
		
		// start the thread that will just keep reading the ultrasonics
		ultrasonicTask = new Thread(new UltrasonicTask(this));
		ultrasonicTask.setDaemon(true);
		ultrasonicTask.start();
	}
	
	public double getRangeInInches(UltrasonicSensor sensor)
	{
		double sensorVoltage = -10;
		synchronized(reading)
		{
			sensorVoltage = reading[sensor.index];
		}
		
		// Convert voltage to inches (and limit it to sensor range capability)
		double distanceInInches = sensorVoltage / sensor.voltsPerInch;
		distanceInInches = (distanceInInches > sensor.maxRangeInches) ? sensor.maxRangeInches : distanceInInches;
		distanceInInches = (distanceInInches < sensor.minRangeInches) ? sensor.minRangeInches : distanceInInches;  

		return distanceInInches;
	}
	
	
	// Class for the thread that cycles through the ultrasonic sensors 
	//   setting the analog mux address and reading the associated sensor data
	private static class UltrasonicTask implements Runnable {
		
		private static final double ULTRASONIC_STABILIZED_READ_TOLERANCE = 0.1;
		private boolean stop = false;
		Ultrasonics ultrasonics;
		
		public UltrasonicTask(Ultrasonics parent) {
			stop = false;
			ultrasonics = parent;
		}

		@Override
		public void run() {

			double distanceInInches;

			double sensorVoltage, sensorVoltageOldRead; // sensor reads - must approximately match when read to be considered valid
			byte spiCommand[] = {(byte)0x00};

			// pulse the DIO to kickstart the ultrasonics into automatically reading ranges
			ultrasonics.ultrasonicKickstartLine.set(false);
//			boolean notInterrupted;
//			do {
//				try { Thread.sleep(1); notInterrupted = true; } catch(Exception e) {notInterrupted = false;};  // sleep for a millisecond
//			} while (!notInterrupted);
//			ultrasonics.ultrasonicKickstartLine.set(true);
//			do {
//				try { Thread.sleep(1); } catch(Exception e) {notInterrupted = false;};  // sleep for a millisecond
//			} while (!notInterrupted);
//			ultrasonics.ultrasonicKickstartLine.set(false);

			ultrasonics.ultrasonicKickstartLine.pulse(RobotMap.NavUltrasonicKickstartLineDigitalOut, UltrasonicKickoffPulseLenght);
			
			////////////////////////////////////////////////////////////////////////////////////////
			/// TASK LOOP //////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////
			double now;	

			while(!stop)
			{
				for(UltrasonicSensor sensor: UltrasonicSensor.values())
				{
					// Write the LT1390 addressing nibble (4 bits) via the SPI bus
					spiCommand[0] = sensor.spiByte;	
					ultrasonics.muxSPI.write(spiCommand, 1); 

					// wait for the reading to stablilze
					try {
						Thread.sleep(5);     // milliseconds
					} catch (InterruptedException e) {
						// DO NOTHING
					}
					
					// read the analog input (two in a row must match to avoid reading on an update)
					sensorVoltage = ultrasonics.muxedUltrasonicInput.getAverageVoltage();
					do
					{
						sensorVoltageOldRead = sensorVoltage;
						sensorVoltage = ultrasonics.muxedUltrasonicInput.getAverageVoltage();
					}  while(Math.abs(sensorVoltage - sensorVoltageOldRead) < ULTRASONIC_STABILIZED_READ_TOLERANCE);

			        // store the range for use
					synchronized(ultrasonics.reading)
					{						
						ultrasonics.reading[sensor.index] = sensorVoltage;
						// uncomment following for lightly averaged sensor readings
//						ultrasonics.reading[sensor.index] = (ultrasonics.reading[sensor.index] + sensorVoltage)/2;
					}
 				}
			}
			////////////////////////////////////////////////////////////////////////////////////////
			/// END TASK LOOP //////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////

		}

		public void stop()
		{
			stop = true;
		}
	}
	private Thread ultrasonicTask;

}
