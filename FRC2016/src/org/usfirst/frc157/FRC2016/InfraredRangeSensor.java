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
public class InfraredRangeSensor extends RangeSensor {
    
    public static class CalibrationData {

        // Values are for close range infrared range sensor
        public double farVoltage = 0.4;
        public double farDistance = 80;
        public double closeVoltage = 3;
        public double closeDistance = 6;

        // Values fot long range infrared range sensor
//        public double farVoltage = 1.4;
//        public double farDistance = 500;
//        public double closeVoltage = 2.5;
//        public double closeDistance = 100;
    }

    private CalibrationData cal;
    private double voltsPerDistance = 1;
    private double minRange = 0;

    public InfraredRangeSensor(int channel, CalibrationData cal) {
        super(channel);
        calibrate(cal);
    }

    public void calibrate(CalibrationData _cal) {
        cal = new CalibrationData();
        voltsPerDistance = (_cal.farVoltage - _cal.closeVoltage) / (_cal.farDistance - _cal.closeDistance);
        minRange = _cal.closeDistance;

        cal.closeVoltage = _cal.closeVoltage;
        cal.farVoltage = _cal.farVoltage;
        cal.closeDistance = _cal.closeDistance;
        cal.farDistance = _cal.farDistance;

        System.out.println("IR Range Sensor Calibration:");
        System.out.println("cal.closeVoltage = " + cal.closeVoltage);
        System.out.println("cal.farVoltage   = " + cal.farVoltage);
        System.out.println("cal.closeDistance= " + cal.closeDistance);
        System.out.println("cal.farDistance  = " + cal.farDistance);
        System.out.println("voltsPerDistance   = " + voltsPerDistance);
        System.out.println("minRange           = " + minRange);
    }
    
    public double pidGet() {
        return getRange();
    }

    public double getRange() {
        return getRange(10);
    }
    
    public double getRange(int samples) {
        double sum = 0;
//        int samples = 100;
        for (int idx = 0; idx < samples; idx++) {
            sum += voltageToDistance(getVoltage());
        }
        double distance = sum / samples;
        return distance;
//        double voltage = getVoltage();
////        System.out.print("v= " + voltage);
//        return voltageToDistance(voltage);
    }

    private double voltageToDistance(double voltage) {
        // Note: Sensor data sheet says inverse distance is linearish to voltage        
        double deltaVoltage = cal.closeVoltage - cal.farVoltage;
        double deltaInvDist = 1 / cal.closeDistance - 1 / cal.farDistance;
        double invDistPerVolt = deltaInvDist / deltaVoltage;

        double invDistance = (voltage - cal.farVoltage) * invDistPerVolt;
        double distance = 1 / invDistance;

        if(distance < cal.closeDistance) { distance = cal.closeDistance;}
        if(distance > cal.farDistance) {distance = cal.farDistance;}
        
        return distance;
    }
    
       public double getMaxRange()
    {
        return cal.farDistance;
    }
}
