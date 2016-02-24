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

import org.usfirst.frc157.FRC2016.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc157.FRC2016.subsystems.*;
import org.usfirst.frc157.FRC2016.subsystems.Arm.Position;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
    public static final int LEFT_JOYSTICK_ID = 1;
    public static final int RIGHT_JOYSTICK_ID = 2;
    public static final int OPERATOR_JOYSTICK_ID = 0;
    public static final int PIT_CONTROLLER_ID = 3;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public Joystick driverLeft;
    private JoystickButton driverRightButtonTrigger; // Button 1 - Trigger
    private JoystickButton driverRightButton2; // Button 2 - Stick down
    private JoystickButton driverRightButton3; // Button 3 - Stick center
    private JoystickButton driverRightButton4; // Button 4 - Stick left
    private JoystickButton driverRightButton5; // Button 5 - Stick right
    private JoystickButton driverRightButton6; // Button 6 - Base Left Away
    private JoystickButton driverRightButton7; // Button 7 - Base Left Close
    private JoystickButton driverRightButton8; // Button 8 - Base Near Left
    private JoystickButton driverRightButton9; // Button 9 - Base Near Right
    private JoystickButton driverRightButton10; // Button 10 - Base Right Close
    private JoystickButton driverRightButton11; // Button 11 - Base RIght Away
    private JoystickButton driverRightButton12; // Button 12
    
    private JoystickAxisButton driverRightX;  // X Axis operation button
    private JoystickAxisButton driverRightY;  // Y Axis operation button
    
    public Joystick driverRight;
    private JoystickButton driverLeftButtonTrigger; // Button 1 - Trigger
    private JoystickButton driverLeftButton2; // Button 2 - Stick down
    private JoystickButton driverLeftButton3; // Button 3 - Stick center
    private JoystickButton driverLeftButton4; // Button 4 - Stick left
    private JoystickButton driverLeftButton5; // Button 5 - Stick right
    private JoystickButton driverLeftButton6; // Button 6 - Base Left Away
    private JoystickButton driverLeftButton7; // Button 7 - Base Left Close
    private JoystickButton driverLeftButton8; // Button 8 - Base Near Left
    private JoystickButton driverLeftButton9; // Button 9 - Base Near Right
    private JoystickButton driverLeftButton10; // Button 10 - Base Right Close
    private JoystickButton driverLeftButton11; // Button 11 - Base RIght Away
    private JoystickButton driverLeftButton12; // Button 12

    private JoystickAxisButton driverLeftX;  // X Axis operation button
    private JoystickAxisButton driverLeftY;  // Y Axis operation button
    
    public Joystick operator;
    private JoystickButton operatorButtonTrigger; // Button 1 - Trigger
    private JoystickButton operatorButton2; // Button 2 - Stick down
    private JoystickButton operatorButton3; // Button 3 - Stick center
    private JoystickButton operatorButton4; // Button 4 - Stick left
    private JoystickButton operatorButton5; // Button 5 - Stick right
    private JoystickButton operatorButton6; // Button 6 - Base Left Away
    private JoystickButton operatorButton7; // Button 7 - Base Left Close
    private JoystickButton operatorButton8; // Button 8 - Base Near Left
    private JoystickButton operatorButton9; // Button 9 - Base Near Right
    private JoystickButton operatorButton10; // Button 10 - Base Right Close
    private JoystickButton operatorButton11; // Button 11 - Base RIght Away
    private JoystickButton operatorButton12; // Button 12
    
    private JoystickPOVButton operatorHatFore;      // Hat Button Forward
    private JoystickPOVButton operatorHatForeRight; // Hat Button Forward Right
    private JoystickPOVButton operatorHatRight;     // Hat Button Right
    private JoystickPOVButton operatorHatAftRight;  // Hat Button Aft Right
    private JoystickPOVButton operatorHatAft;       // Hat Button Aft
    private JoystickPOVButton operatorHatAftLeft;   // Hat Button Aft Left
    private JoystickPOVButton operatorHatLeft;      // Hat Button Left
    private JoystickPOVButton operatorHatForeLeft;  // Hat Button Forward Left
    
    private JoystickAxisButton operatorX;  // X Axis operation button
    private JoystickAxisButton operatorY;  // Y Axis operation button
    private JoystickAxisButton operatorZ;  // X Axis operation button

//    //TEMP: Disable Logitech
//    public LogitechController logitechDriver;
//    private LogitechControllerButton logitechDriverButtonLeftB; // Left Button (Above Trigger)
//    private LogitechControllerButton logitechDriverButtonRightB; // Right Button (Above Trigger)
//    private LogitechControllerButton logitechDriverButtonLeftTop; // Left Stick Pressing
//    private LogitechControllerButton logitechDriverButtonRightTop; // Right Stick Pressing
//    private LogitechControllerButton logitechDriverButtonA; // A Button (Green)
//    private LogitechControllerButton logitechDriverButtonB; // B Button (Red)
//    private LogitechControllerButton logitechDriverButtonX; // X Button (Blue)
//    private LogitechControllerButton logitechDriverButtonY; // Y Button (Yellow)
//    private LogitechControllerButton logitechDriverButtonStart; // Start Button
//    private LogitechControllerButton logitechDriverButtonBack; // Back Button
//    private LogitechControllerButton logitechDriverButtonLeftTrigger; // Left Trigger (Used as button)
//    private LogitechControllerButton logitechDriverButtonRightTrigger; // Right Trigger (Used as button)
//    private LogitechControllerButton logitechDriverButtonGameUp; // Game Pad Up
//    private LogitechControllerButton logitechDriverButtonGameDown; // Game Pad Down
//    private LogitechControllerButton logitechDriverButtonGameLeft; // Game Pad Left
//    private LogitechControllerButton logitechDriverButtonGameRight; // Game Pad Right

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        /////////////////////////////////////////////////////////////////
        driverLeft = new Joystick(LEFT_JOYSTICK_ID);
        driverRight = new Joystick(RIGHT_JOYSTICK_ID);
        operator = new Joystick(OPERATOR_JOYSTICK_ID);
//        //TEMP: Disable Logitech
//        logitechDriver = new LogitechController(PIT_CONTROLLER_ID);

        // -----------------------------------------//
        // -----------------------------------------//

        // Instantiate all of the buttons on the 2 joysticks and the logitech controller
        driverLeftButtonTrigger = new JoystickButton(driverLeft, 1);
        driverLeftButton2 = new JoystickButton(driverLeft, 2);
        driverLeftButton3 = new JoystickButton(driverLeft, 3);
        driverLeftButton4 = new JoystickButton(driverLeft, 4);
        driverLeftButton5 = new JoystickButton(driverLeft, 5);
        driverLeftButton6 = new JoystickButton(driverLeft, 6);
        driverLeftButton7 = new JoystickButton(driverLeft, 7);
        driverLeftButton8 = new JoystickButton(driverLeft, 8);
        driverLeftButton9 = new JoystickButton(driverLeft, 9);
        driverLeftButton10 = new JoystickButton(driverLeft, 10);
        driverLeftButton11 = new JoystickButton(driverLeft, 11);
        driverLeftButton12 = new JoystickButton(driverLeft, 12);

        driverLeftX = new JoystickAxisButton(driverLeft, 0, JoystickAxisButton.Direction.BOTH, 0.1);  // X Axis operation button
        driverLeftY = new JoystickAxisButton(driverLeft, 1, JoystickAxisButton.Direction.BOTH, 0.1);  // Y Axis operation button
        
        driverRightButtonTrigger = new JoystickButton(driverRight, 1);
        driverRightButton2 = new JoystickButton(driverRight, 2);
        driverRightButton3 = new JoystickButton(driverRight, 3);
        driverRightButton4 = new JoystickButton(driverRight, 4);
        driverRightButton5 = new JoystickButton(driverRight, 5);
        driverRightButton6 = new JoystickButton(driverRight, 6);
        driverRightButton7 = new JoystickButton(driverRight, 7);
        driverRightButton8 = new JoystickButton(driverRight, 8);
        driverRightButton9 = new JoystickButton(driverRight, 9);
        driverRightButton10 = new JoystickButton(driverRight, 10);
        driverRightButton11 = new JoystickButton(driverRight, 11);
        driverRightButton12 = new JoystickButton(driverRight, 12);

        driverRightX = new JoystickAxisButton(driverLeft, 0, JoystickAxisButton.Direction.BOTH, 0.1);  // X Axis operation button
        driverRightY = new JoystickAxisButton(driverLeft, 1, JoystickAxisButton.Direction.BOTH, 0.1);  // Y Axis operation button

        operatorButtonTrigger = new JoystickButton(operator, 1);
        operatorButton2 = new JoystickButton(operator, 2);
        operatorButton3 = new JoystickButton(operator, 3);
        operatorButton4 = new JoystickButton(operator, 4);
        operatorButton5 = new JoystickButton(operator, 5);
        operatorButton6 = new JoystickButton(operator, 6);
        operatorButton7 = new JoystickButton(operator, 7);
        operatorButton8 = new JoystickButton(operator, 8);
        operatorButton9 = new JoystickButton(operator, 9);
        operatorButton10 = new JoystickButton(operator, 10);
        operatorButton11 = new JoystickButton(operator, 11);
        operatorButton12 = new JoystickButton(operator, 12);
    
        operatorHatFore = new JoystickPOVButton(operator, 0);
        operatorHatForeRight = new JoystickPOVButton(operator, 45);
        operatorHatRight = new JoystickPOVButton(operator, 90);
        operatorHatAftRight = new JoystickPOVButton(operator, 135);
        operatorHatAft = new JoystickPOVButton(operator, 180);
        operatorHatAftLeft = new JoystickPOVButton(operator, 225);
        operatorHatLeft = new JoystickPOVButton(operator, 270);
        operatorHatForeLeft = new JoystickPOVButton(operator, 315);

        operatorX = new JoystickAxisButton(driverLeft, 0, JoystickAxisButton.Direction.BOTH, 0.1);  // X Axis operation button
        operatorY = new JoystickAxisButton(driverLeft, 1, JoystickAxisButton.Direction.BOTH, 0.1);  // Y Axis operation button
        operatorZ = new JoystickAxisButton(driverLeft, 2, JoystickAxisButton.Direction.BOTH, 0.1);  // Y Axis operation button

//        //TEMP: Disable Logitech
//        logitechDriverButtonLeftB = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonLeftB.value);
//        logitechDriverButtonRightB = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonRightB.value);
//        logitechDriverButtonLeftTop = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonLeftTop.value);
//        logitechDriverButtonRightTop = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonRightTop.value);
//        logitechDriverButtonA = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonA.value);
//        logitechDriverButtonB = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonB.value);
//        logitechDriverButtonX = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonX.value);
//        logitechDriverButtonY = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonY.value);
//        logitechDriverButtonStart = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonStart.value);
//        logitechDriverButtonBack = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonBack.value);
//        logitechDriverButtonLeftTrigger = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonLeftTrigger.value);
//        logitechDriverButtonRightTrigger = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonRightTrigger.value);
//        logitechDriverButtonGameUp = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonGameUp.value);
//        logitechDriverButtonGameDown = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonGameDown.value);
//        logitechDriverButtonGameLeft = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonGameLeft.value);
//        logitechDriverButtonGameRight = new LogitechControllerButton(logitechDriver, LogitechController.ButtonType.kButtonGameRight.value);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        
        // Put print commands on all buttons
        driverLeftButtonTrigger.whileHeld(new GrabBoulderManual());
        driverLeftButton2.whenPressed(new AutoDriveTurnToHeading(180));
        driverLeftButton3.whenPressed(new AutoDriveTurnToHeading(0));
        driverLeftButton4.whenPressed(new AutoDriveTurnToHeading(-90));
        driverLeftButton5.whenPressed(new AutoDriveTurnToHeading(90));
        driverLeftButton6.whenPressed(new PrintButton("L Button 6"));
        driverLeftButton7.whenPressed(new PrintButton("L Button 7"));
        driverLeftButton8.whenPressed(new PrintButton("L Button 8"));
        driverLeftButton9.whenPressed(new PrintButton("L Button 9"));
        driverLeftButton10.whenPressed(new PrintButton("L Button 10"));
        driverLeftButton11.whenPressed(new PrintButton("L Button 11"));
        driverLeftButton12.whenPressed(new PrintButton("L Button 12"));

//        driverLeftX.whenPressed(new AutoDriveStop("L X Motion"));
//        driverLeftY.whenPressed(new AutoDriveStop("L Y Motion"));

        driverRightButtonTrigger.whileHeld(new GrabBoulderManual());
        driverRightButton2.whenPressed(new ArmShoulderSetAngle(Position.GAME_START.angle())); 
        driverRightButton3.whenPressed(new ArmShoulderSetAngle(Position.LOW_BAR_TRAVEL.angle()));
        driverRightButton4.whenPressed(new ArmShoulderSetAngle(Position.PREPARE_FOR_BOULDER.angle()));
        driverRightButton5.whenPressed(new ArmShoulderSetAngle(Position.GRAB_BOULDER.angle()));
        driverRightButton6.whenPressed(new PrintButton("R Button 6"));
        driverRightButton7.whenPressed(new PrintButton("R Button 7"));
        driverRightButton8.whenPressed(new PrintButton("R Button 8"));
        driverRightButton9.whenPressed(new PrintButton("R Button 9"));
        driverRightButton10.whenPressed(new PrintButton("R Button 10"));
        driverRightButton11.whenPressed(new PrintButton("R Button 11"));
        driverRightButton12.whenPressed(new PrintButton("R Button 12"));
        
//        driverRightX.whenPressed(new AutoDriveStop("R X Motion"));
//        driverRightY.whenPressed(new AutoDriveStop("R Y Motion"));
        
        operatorButtonTrigger.whileHeld(new GrabBoulderManual());     // Trigger
        operatorButton2.whenPressed(new LaunchBoulder());             // Thumb Button
        operatorButton3.whenPressed(new ArmShoulderSetAngle(Position.PREPARE_FOR_BOULDER.angle())); // Button 3
        operatorButton4.whenPressed(new ArmShoulderSetAngle(Position.LOW_BAR_TRAVEL.angle()));      // Button 4
        operatorButton5.whenPressed(new ArmShoulderSetAngle(Position.TOWER_SCALE.angle()));         // Button 5
        operatorButton6.whenPressed(new ArmShoulderSetAngle(Position.FRENCH_FRIES_DOWN.angle()));   // Button 6
        operatorButton7.whileHeld(new PrintIMUOutput("IMU"));
        operatorButton8.whenPressed(new PrintButton("O Button 10"));
//        operatorButton9.whenPressed(new TurnToHeading(0));
        operatorButton10.whileHeld(new PrintIRRange());
//        operatorButton11.whenPressed(new TurnToHeading(90));
//        operatorButton12.whenPressed(new TurnToHeading(-90));
        
        operatorHatFore.whileHeld(new ArmShoulderManual(ArmShoulderManual.Direction.UP));
        operatorHatForeRight.whenPressed(new PrintButton("O  Hat Fore Right"));
        operatorHatRight.whenPressed(new PrintButton("O  Hat Right"));
        operatorHatAftRight.whenPressed(new PrintButton("O  Hat Aft Right"));
        operatorHatAft.whileHeld(new ArmShoulderManual(ArmShoulderManual.Direction.DOWN));
        operatorHatAftLeft.whenPressed(new PrintButton("O  Hat Aft Left"));
        operatorHatLeft.whenPressed(new PrintButton("O  Hat Left"));
        operatorHatForeLeft.whenPressed(new PrintButton("O  Hat Fore Left"));

//        operatorX.whenPressed(new AutoDriveStop("O X Motion"));
//        operatorY.whenPressed(new AutoDriveStop("O Y Motion"));
//        operatorZ.whenPressed(new AutoDriveStop("O Z Motion"));

//        //TEMP: Disable Logitech
//        logitechDriverButtonLeftB.whenPressed(new PrintButton("P Button LeftB"));
//        logitechDriverButtonRightB.whenPressed(new PrintButton("P Button RightB"));
//        logitechDriverButtonLeftTop.whenPressed(new PrintButton("P Button LeftTop"));
//        logitechDriverButtonRightTop.whenPressed(new PrintButton("P Button RightTop"));
//        logitechDriverButtonA.whenPressed(new PrintButton("P Button A"));
//        logitechDriverButtonB.whenPressed(new PrintButton("P Button B"));
//        logitechDriverButtonX.whenPressed(new PrintButton("P Button X"));
//        logitechDriverButtonY.whenPressed(new PrintButton("P Button Y"));
//        logitechDriverButtonStart.whenPressed(new PrintButton("P Button Start"));
//        logitechDriverButtonBack.whenPressed(new PrintButton("P Button Back"));
//        logitechDriverButtonLeftTrigger.whenPressed(new PrintButton("P Button LeftTrigger"));
//        logitechDriverButtonRightTrigger.whenPressed(new PrintButton("P Button RightTrigger"));
//        logitechDriverButtonGameUp.whenPressed(new PrintButton("P Button GameUp"));
//        logitechDriverButtonGameDown.whenPressed(new PrintButton("P Button GameDown"));
//        logitechDriverButtonGameLeft.whenPressed(new PrintButton("P Button GameLeft"));
//        logitechDriverButtonGameRight.whenPressed(new PrintButton("P Button GameRight"));

        SmartDashboard.putData("PrintButton", new PrintButton("SmartDashboard - PrintButton"));
        SmartDashboard.putData("TeleopDrive", new TeleopDrive());
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getArcadeStick() {
        return operator;
    }

    public Joystick getLeftTankStick() {
        return driverLeft;
    }

    public Joystick getRightTankStick() {
        return driverRight;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

