/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//@Autonomous(name="Time Slide Op Mode", group="Pushbot")
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "JK Driver Mecanum", group = "K9Bot")
public class JK_Driver_Mecanum extends LinearOpMode {


    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    joyStick g1 = new joyStick();
    joyStick g2 = new joyStick();
    botMotors mDrive = new botMotors();
    // Define class members
    final long MINORFRAME = 50;
    final long TELEMETRYPERIOD = 1000;

    int rightMotorPos;
    int lefMotorPos;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Time keeping, prime the variables
        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;
        // long squareDur       = 0;

        // Variables to store actuator commands
        double leftDriveCmd = 0;
        double rightDriveCmd = 0;
        double leftRearCmd = 0;
        double rightRearCmd = 0;
        double leftDriveCrab = 0;
        double rightDriveCrab = 0;
        double leftRearCrab = 0;
        double rightRearCrab = 0;
       /* double loaderMotorCmd      = 0;
        double rampMotorCmd        = 0;
        double extensionMotorCmd   = 0;
        double relicExtendMotorCmd = 0;
        double relicPivotServo     = 0;
        double relicClawServo      = 0;
*/

        float g1_X_Threshold = (float) 0.4;
        float g1_Y_Threshold = (float) 0.1;
        float g1_Crab_Threshold = (float) 0.3;
        float g2_X_Threshold = (float) 0.1;
        float g2_Y_Threshold = (float) 0.1;

        // State variables
       /* boolean rampDeployed = false;  //make sticky
        boolean rampBackoff  = false;  // make sticky
        int     extensionCount = 0;
        int     backoffCount   = 5;    //Backoff for five ticks
*/


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();

        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
/* ***********************************************************************************************
   *****************************                CODE          ************************************
   ************************************************************************************************/


        /* ************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > MINORFRAME) {
                LastSensor = CurrentTime;
                //waitForStart();

                // send the info back to driver station using telemetry function.
                // if the digital channel returns true it's HIGH and the button is unpressed.
            }
            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > MINORFRAME) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.ROG
                rightMotorPos = robot.rightDrive.getCurrentPosition();
                lefMotorPos = robot.leftDrive.getCurrentPosition();

            }
            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:
             *         g1_LeftX    g1_RightX
             *         g1_LeftY    g1_RightY
             *         g1_a (gamepad A)
             *         g1_b (gamepad B)
             ****************************************************/
            if (CurrentTime - LastController > MINORFRAME) {
                LastController = CurrentTime;
                g1.LeftX = gamepad1.left_stick_x;
                g1.LeftY = gamepad1.left_stick_y;
                g1.RightX = gamepad1.right_stick_x;
                g1.RightY = gamepad1.right_stick_y;

                g2.LeftX = gamepad2.left_stick_x;
                g2.RightX = gamepad2.right_stick_x;
                g2.LeftY = gamepad2.left_stick_y;/*dim and stuff temp*/

                g2.RightY = gamepad2.right_stick_y;
                //Get controller inputs for buttons and bumpers, may need to
                //add debounce if spurious button push would cause bad
                //performance.
                g1.A = gamepad1.a;
                g1.B = gamepad1.b;
                g1.X = gamepad1.x;
                g1.Y = gamepad1.y;
                g1.DD = gamepad1.dpad_down;
                g1.DL = gamepad1.dpad_left;
                g1.DR = gamepad1.dpad_right;
                g1.DU = gamepad1.dpad_up;
                g1.RB = gamepad1.right_bumper;
                g1.LB = gamepad1.left_bumper;
                g1.RT = gamepad1.right_trigger;
                g1.LT = gamepad1.left_trigger;

                g2.A = gamepad2.a;
                g2.B = gamepad2.b;
                g2.X = gamepad2.x;
                g2.Y = gamepad2.y;
                g2.DD = gamepad2.dpad_down;
                g2.DL = gamepad2.dpad_left;
                g2.DR = gamepad2.dpad_right;
                g2.DU = gamepad2.dpad_up;
                g2.RB = gamepad2.right_bumper;
                g2.LB = gamepad2.left_bumper;
                g2.RT = gamepad2.right_trigger;
                g2.LT = gamepad2.left_trigger;
            }

        /*  ***********************************************************************
         ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
         ***************************************************************************/



        /* ********************************************************************************/
        /* vvvvvvvvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/

            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
            if (CurrentTime - LastNav > MINORFRAME) {
                LastNav = CurrentTime;

                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.
                double driveMax = 1;
                double driveMin = -1;
                //double rampMin       = -0.4;
                //double rampMax       = 0.4;
                double extensionMin = -1;
                double extensionMax = 1;
                double feederMin = -1;
                double feederMax = 1;


                leftDriveCmd = 0;
                rightDriveCmd = 0;
                leftRearCmd = 0;
                rightRearCmd = 0;
                leftDriveCrab = 0;
                rightDriveCrab = 0;
                leftRearCrab = 0;
                rightRearCrab = 0;


                if (Math.abs(g1.LeftX) < g1_X_Threshold) {
                    //Pivot Right
                    leftDriveCmd = Range.clip(g1.LeftY * g1.LeftY * g1.LeftY, driveMin, driveMax);
                    rightDriveCmd = leftDriveCmd;
                    leftRearCmd = leftDriveCmd;
                    rightRearCmd = leftDriveCmd;
                }

                if (Math.abs(g1.LeftX) > g1_X_Threshold) {
                    //Pivot Left
                    leftDriveCmd = Range.clip(g1.LeftX * g1.LeftX * g1.LeftX, driveMin, driveMax);
                    rightDriveCmd = -1 * Range.clip(g1.LeftX * g1.LeftX * g1.LeftX, driveMin, driveMax);
                    leftRearCmd = leftDriveCmd;
                    rightRearCmd = rightDriveCmd;
                }

                if ((Math.abs(g1.RightY) > g1_Crab_Threshold) || (Math.abs(g1.RightX) > g1_Crab_Threshold)) {

                    if (Math.abs(g1.RightX) < g1_Crab_Threshold) {   //Forward, backward
                        leftDriveCrab = Range.clip(g1.RightY * g1.RightY * g1.RightY, driveMin, driveMax);
                        rightDriveCrab = leftDriveCrab;
                        leftRearCrab = leftDriveCrab;
                        rightRearCrab = leftDriveCrab;
                    } else {
                        leftDriveCrab = Range.clip(g1.RightX * g1.RightX * g1.RightX, driveMin, driveMax);
                        rightDriveCrab = -1 * leftDriveCrab;
                        leftRearCrab = -1 * leftDriveCrab;
                        rightRearCrab = leftDriveCrab;
                    }

                }                    // END NAVIGATION


        /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^*/
        /* ********************************************************************************/



        /*  ***********************************************************************
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
         *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/


            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
                if (CurrentTime - LastServo > MINORFRAME) {
                    LastServo = CurrentTime;
                    if (g1.A) {
                    } else {
                    }


                    // No servos on the robot
                }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
                if (CurrentTime - LastMotor > MINORFRAME) {
                    LastMotor = CurrentTime;

                    robot.leftDrive.setPower((rightDriveCmd + rightDriveCrab));
                    robot.rightDrive.setPower((leftDriveCmd + leftDriveCrab));
                    robot.leftRear.setPower((rightRearCmd + rightRearCrab));
                    robot.rightRear.setPower((leftRearCmd + leftRearCrab));

                    //robot.extensionMotor.setPower(extensionMotorCmd);
                    //robot.rampMotor.setPower(rampMotorCmd);
                    //robot.loaderMotor.setPower(loaderMotorCmd);
                }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

                if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                    LastTelemetry = CurrentTime;
                    telemetry.addData("Left Motor Power:       ", leftDriveCmd + leftDriveCrab);
                    telemetry.addData("Left Rear  Power:       ", leftRearCmd + leftRearCrab);
                    telemetry.addData("Right Motor Power:      ", rightDriveCmd + rightDriveCrab);
                    telemetry.addData("Right Rear  Power:      ", rightRearCmd + rightRearCrab);
                    telemetry.addData("Left Trigger  ", g1.LT);
                    telemetry.addData("Right Trigger ", g1.RT);
                    telemetry.addData("Left Bumper   ", g1.LB);
                    telemetry.addData("Right Bumper  ", g1.RB);
                    telemetry.addData("A             ", g1.A);
                    telemetry.addData("B             ", g1.B);
                    telemetry.update();
                }
            }

            }// end while opmode is active

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            //robot.extensionMotor.setPower(0);
            //robot.rampMotor.setPower(0);
            //robot.loaderMotor.setPower(0);
        }
    }

