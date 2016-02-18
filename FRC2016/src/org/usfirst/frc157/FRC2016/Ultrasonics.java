package org.usfirst.frc157.FRC2016;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class Ultrasonics {

	private final static float UltrasonicKickoffPulseLenght = (float) 0.00001; // seconds 
//	private final static float UltrasonicKickoffPulseLenght = (float) 0.001; // seconds 
	private final static double DoubleReadTolerance = (double)0.005; //volts
	
	public enum UltrasonicSensor
	{
		// Name       Index, muxAddr, SPI Byte,   V/in,   min, max    Min, Max ensor range are in inches
		FRONT_LEFT    (0,    0,       (byte)0x80, 0.0098, 6.0, 254.0),
		FRONT_RIGHT   (1,    1,       (byte)0x90, 0.0098, 6.0, 254.0),
		RIGHT_FRONT   (2,    2,       (byte)0xA0, 0.0098, 6.0, 254.0),
		RIGHT_REAR    (3,    3,       (byte)0xB0, 0.0098, 6.0, 254.0),
		REAR_RIGHT    (4,    4,       (byte)0xC0, 0.0098, 6.0, 254.0),
		REAR_LEFT     (5,    5,       (byte)0xD0, 0.0098, 6.0, 254.0),
		LEFT_REAR     (6,    6,       (byte)0xE0, 0.0098, 6.0, 254.0),
		LEFT_FRONT    (7,    7,       (byte)0xF0, 0.0098, 6.0, 254.0);

		private final int    index;
		private final int    muxAddress;
		private final byte   spiByte;
		private final double voltsPerInch;
		private final double minRangeInches;
		private final double maxRangeInches;

		UltrasonicSensor(int index, int muxAddress, byte spiByte, double voltsPerInch, double minRangeInches, double maxRangeInches) {
			this.index = index;
			this.muxAddress = muxAddress;
			this.spiByte = spiByte;
			this.voltsPerInch = voltsPerInch;
			this.minRangeInches = minRangeInches;
			this.maxRangeInches = maxRangeInches;
		}
	}

	private AnalogInput muxedUltrasonicInput;
	private DigitalOutput ultrasonicKickstartLine;
	private SPI muxSPI;

    double ranges[] = new double[8];
    
	public Ultrasonics(int analogInput, int kickstartLineDigtalOutput, Port spiPort)
	{
		for(int idx=0; idx<8; idx++)
		{
			ranges[idx]=-1;
		}
		
		if(muxedUltrasonicInput == null)
		{
			muxedUltrasonicInput = new AnalogInput(analogInput);
		}
		if(ultrasonicKickstartLine == null)
		{
			ultrasonicKickstartLine = new DigitalOutput(kickstartLineDigtalOutput);
		}
		if(muxSPI == null)
		{
			muxSPI = new SPI(spiPort);
		}
		

		muxSPI.setClockRate(1000000);
		muxSPI.setChipSelectActiveHigh();
		muxSPI.setClockActiveHigh();
		muxSPI.setMSBFirst();
		muxSPI.setSampleDataOnRising();
		
		byte toSend[] = {(byte)0xAA, (byte)0x05};
		muxSPI.write(toSend, 2);  // Fake Data
		
		// start the thread that will just keep reading the ultrasonics
		ultrasonicTask = new Thread(new UltrasonicTask(this));
		ultrasonicTask.setDaemon(true);
		ultrasonicTask.start();
	}
	
	public double getRange(UltrasonicSensor sensor)
	{
		double range = ranges[sensor.index];
		double checkRange = ranges[sensor.index];
		
		while(range != checkRange)
		{
			checkRange = range;
			range = ranges[sensor.index];
		}
		return range;
	}
	
	private static class UltrasonicTask implements Runnable {
		private boolean stop = false;
		Ultrasonics ultrasonics;
		public UltrasonicTask(Ultrasonics parent) {
			stop = false;
			ultrasonics = parent;
		}

		@Override
		public void run() {
			int idx = 0;
			double distanceInInches;
			double sensorVoltage, sensorVoltageOldRead; // sensor reads - must mach when read to be considered valid
			byte data[] = {(byte)0x00};
			// Set up the ultrasonic scan order
			byte address[] = {
					Ultrasonics.UltrasonicSensor.FRONT_LEFT.spiByte,
					Ultrasonics.UltrasonicSensor.FRONT_LEFT.spiByte,
					Ultrasonics.UltrasonicSensor.FRONT_RIGHT.spiByte,
					Ultrasonics.UltrasonicSensor.RIGHT_FRONT.spiByte,
					Ultrasonics.UltrasonicSensor.RIGHT_REAR.spiByte,
					Ultrasonics.UltrasonicSensor.REAR_RIGHT.spiByte,
					Ultrasonics.UltrasonicSensor.REAR_LEFT.spiByte,
					Ultrasonics.UltrasonicSensor.LEFT_REAR.spiByte,
					Ultrasonics.UltrasonicSensor.LEFT_FRONT.spiByte
					};

			// pulse the DIO to kickstart the ultrasonics into automatically reading ranges
			ultrasonics.ultrasonicKickstartLine.set(false);
			boolean notInterrupted;
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
			while(!stop)
			{
				for(UltrasonicSensor sensor: UltrasonicSensor.values())
				{
					// TODO verify that the SPI command is correct for the LTC1390 8 Channel Analog Mux
					data[0] = address[idx%8];	
					idx++;
					if(idx == 8) {idx = 0;}
					ultrasonics.muxSPI.write(data, 1); 
					
					// read the analog input (two in a row must match to avoid reading on an update)
					sensorVoltage = ultrasonics.muxedUltrasonicInput.getVoltage();
					do 
					{
						sensorVoltageOldRead = sensorVoltage;
						sensorVoltage = ultrasonics.muxedUltrasonicInput.getVoltage();
					} while (Math.abs(sensorVoltage - sensorVoltageOldRead) > DoubleReadTolerance);
					
					// Convert voltage to inches (and limit it to sensor range capability)
					distanceInInches = sensorVoltage / sensor.voltsPerInch;
			        distanceInInches = (distanceInInches > sensor.maxRangeInches) ? sensor.maxRangeInches : distanceInInches;
			        distanceInInches = (distanceInInches < sensor.minRangeInches) ? sensor.minRangeInches : distanceInInches;  
			        
			        // store the range for use
			        ultrasonics.ranges[sensor.index] = distanceInInches;
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
