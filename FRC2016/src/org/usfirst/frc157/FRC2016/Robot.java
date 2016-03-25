// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2016;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc157.FRC2016.subsystems.Navigation;
import org.usfirst.frc157.FRC2016.AnalogSelectSwitch.SwitchPosition;
import org.usfirst.frc157.FRC2016.commands.*;
import org.usfirst.frc157.FRC2016.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    //Global Commands 
    Command autonomousCommand;
    Command armChainWatcherCommand; 
    
    // Camera on robot to display on the driver station
    public CameraServer camera;

    //OPerator Interface
    public static OI oi;

    //SubSystems
    public static Drive drive;
    public static Arm arm;
    public static BallHandler ballHandler;
    public static Navigation navigation;
    
    
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	RobotMap.init();
    	
        // Surround with a try/catch so that if a network error occurs, robot does not crash
        try
        {
            // Sets up the camera for display on the driver station
            camera = CameraServer.getInstance();
            camera.setQuality(50);
            camera.startAutomaticCapture("cam0");
        }
        catch (Exception E)
        {
            // Debug print
            System.out.println("Problem with camera!");
        }
        
        //SubSystems
        drive = new Drive();
        arm = new Arm();
        ballHandler = new BallHandler();
        //navigation = new Navigation();
        
        System.out.println("Robot.robotInit() Subsystems Initialized");

        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        
        //Commands
        armChainWatcherCommand = new ArmChainWatcher();
        //Note: autonomousCommand is created in autonomousInit() because it reads the AnalogSwitch when it is created.
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
    	Robot.arm.disableControl();
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {
        //Robot.navigation.resetZeroHeading();
    	
        //Note: autonomousCommand is created in autonomousInit() because it reads the AnalogSwitch when it is created.
        autonomousCommand =new AutonomousCommand();
        if (autonomousCommand != null) autonomousCommand.start();
        
        if (armChainWatcherCommand != null) armChainWatcherCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();

        if (armChainWatcherCommand != null) armChainWatcherCommand.start();

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
