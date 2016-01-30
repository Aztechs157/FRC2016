/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc157.FRC2016;

/**
 *
 * @author mattkahn
 */
public class UltrasonicRangeSensor extends RangeSensor {

    public static class CalibrationData {

        // Values are for close range infrared range sensor
        public double voltsPerInch = 0.0098;
        public double minRangeInches = 6;
        public double maxRangeInches = 254;

    }

    private CalibrationData cal;
    
    public UltrasonicRangeSensor(int channel, CalibrationData cal) {
        super(channel);
        calibrate(cal);
    }

    private void calibrate(CalibrationData _cal)
    {
        cal = new CalibrationData();
        cal.maxRangeInches = _cal.maxRangeInches;
        cal.minRangeInches = _cal.minRangeInches;
        cal.voltsPerInch = _cal.voltsPerInch;
        
        System.out.println("Ultrasonic Range Sensor Calibration:");
        System.out.println("cal.maxRangeInches = " + cal.maxRangeInches);
        System.out.println("cal.minRangeInches   = " + cal.minRangeInches);
        System.out.println("cal.voltsPerInch= " + cal.voltsPerInch);
    }
    
    public double pidGet() {
         return getRange();
    }

    public double getRange() {
         return getRange(1);
    }

    public double getRange(int samples) {
        double sum = 0;
//        int samples = 100;
        for (int idx = 0; idx < samples; idx++) {
            sum += getVoltage();
        }
        double voltage = sum / samples;
        
        double distance = voltage / cal.voltsPerInch;
        distance = (distance > cal.maxRangeInches) ? cal.maxRangeInches : distance;
        distance = (distance < cal.minRangeInches) ? cal.minRangeInches : distance;        
        return distance * 2.54;
    }

    public double getMaxRange()
    {
        return cal.maxRangeInches * 2.54;
    }
    
    public double getVoltage()
    {
    	double voltage1 = super.getVoltage();
    	double voltage2 = super.getVoltage();
    	while(voltage1 != voltage2)
    	{
    		voltage1 = voltage2;
    		voltage2 = super.getVoltage();
    	}
		return voltage1;
    }
}
