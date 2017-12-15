/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


//@Autonomous(name="Time Slide Op Mode", group="Pushbot")
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "JK Driver Controlledf", group = "K9Bot")
public class JK_DriverOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    //private judenKiBot robot = new judenKiBot();   // Use a Pushbot's hardware
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    //   private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    //   private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    //   private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //   private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //           (WHEEL_DIAMETER_INCHES * 3.1415);
    //   private static final double DRIVE_SPEED = 0.6;
    //   private static final double TURN_SPEED = 0.5;






    // Define class members
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;

    final double PROPGAIN = 0.6;
    final double INTGAIN = 0.3;
    final double DERGAIN = 0.1;
    final long PIDMAXDUR = 3;


    int rightMotorPos;
    int lefMotorPos;


    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    public double simplePID(double err, double duration, double prevErr) {
        double pidmin = -.7;
        double pidmax = 0.7;
        double propTerm = Range.clip(PROPGAIN * err, pidmin, pidmax);
        double intTerm = Range.clip(duration / PIDMAXDUR, pidmin, pidmax) * INTGAIN;
        double derTerm = Range.clip((err - prevErr), pidmin, pidmax) * DERGAIN;
        return (Range.clip(propTerm + intTerm + derTerm, pidmin, pidmax));
    }

    @Override

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //Time keeping, prime the variables
        long CurrentTime     = System.currentTimeMillis();
        long LastSensor      = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo       = CurrentTime + 10;
        long LastNav         = CurrentTime + 15;
        long LastMotor       = CurrentTime + 20;
        long LastController  = CurrentTime + 7;
        long LastTelemetry   = CurrentTime + 17;


        // variables to store controller inputs.
        float g1_LeftX     = 0;
        float g1_LeftY     = 0;
        float g1_RightX    = 0;
        float g1_RightY    = 0;
        boolean g1_A       = false;
        boolean g1_B       = false;
        boolean g1_X       = false;
        boolean g1_Y       = false;
        boolean g1_LB      = false;
        boolean g1_RB      = false;
        float   g1_LT      = 0;
        float   g1_RT      = 0;
        boolean g1_DU      = false;
        boolean g1_DD      = false;
        boolean g1_DL      = false;
        boolean g1_DR      = false;

        float g2_LeftX     = 0;
        float g2_LeftY     = 0;
        float g2_RightX    = 0;
        float g2_RightY    = 0;
        boolean g2_A       = false;
        boolean g2_B       = false;
        boolean g2_X       = false;
        boolean g2_Y       = false;
        boolean g2_LB      = false;
        boolean g2_RB      = false;
        float   g2_LT      = 0;
        float   g2_RT      = 0;
        boolean g2_DU      = false;
        boolean g2_DD      = false;
        boolean g2_DL      = false;
        boolean g2_DR      = false;


        // Variables to store actuator commands
        double leftDriveCmd       = 0;
        double rightDriveCmd      = 0;
        double loaderMotorCmd     = 0;
        double rampMotorCmd       = 0;
        double extensionMotorCmd  = 0;

        // State variables
        boolean rampDeployed = false;  //make sticky
        boolean rampBackoff  = false;  // make sticky
        int     extensionCount = 0;
        int     backoffCount   = 5;    //Backoff for five ticks




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
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                //waitForStart();

                // send the info back to driver station using telemetry function.
                // if the digital channel returns true it's HIGH and the button is unpressed.
                if (robot.extensionTouch.isPressed()) {
                    telemetry.addData("Ramp Extension", "Is Pressed");
                    rampDeployed = true;
                } else {
                    telemetry.addData("Ramp Extension", "Is Not Pressed");
                }
            }
            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
                if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                    LastEncoderRead = CurrentTime;
                    // We want to READ the Encoders here
                    //    ONLY set the motors in motion in ONE place.ROG
                    rightMotorPos = robot.rightDrive.getCurrentPosition();
                    lefMotorPos   = robot.leftDrive.getCurrentPosition();

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
                if (CurrentTime - LastController > CONTROLLERPERIOD) {
                    LastController = CurrentTime;
                    g1_LeftX = gamepad1.left_stick_x;
                    g1_LeftY = gamepad1.left_stick_y;
                    g2_LeftY = gamepad2.left_stick_y;
                    g1_RightX = gamepad1.right_stick_x;
                    g1_RightY = gamepad1.right_stick_y;
                    g2_RightY = gamepad2.right_stick_y;
                    //Get controller inputs for buttons and bumpers, may need to
                    //add debounce if spurious button push would cause bad
                    //performance.
                    g1_A = gamepad1.a;
                    g1_B = gamepad1.b;
                    g1_X = gamepad1.x;
                    g1_Y = gamepad1.y;
                    g1_DD = gamepad1.dpad_down;
                    g1_DL = gamepad1.dpad_left;
                    g1_DR = gamepad1.dpad_right;
                    g1_DU = gamepad1.dpad_up;
                    g1_RB = gamepad1.right_bumper;
                    g1_LB = gamepad1.left_bumper;
                    g1_RT = gamepad1.right_trigger;
                    g1_LT = gamepad1.left_trigger;

                    g2_A = gamepad2.a;
                    g2_B = gamepad2.b;
                    g2_X = gamepad2.x;
                    g2_Y = gamepad2.y;
                    g2_DD = gamepad2.dpad_down;
                    g2_DL = gamepad2.dpad_left;
                    g2_DR = gamepad2.dpad_right;
                    g2_DU = gamepad2.dpad_up;
                    g2_RB = gamepad2.right_bumper;
                    g2_LB = gamepad2.left_bumper;
                    g2_RT = gamepad2.right_trigger;
                    g2_LT = gamepad2.left_trigger;
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
                    if (CurrentTime - LastNav > NAVPERIOD) {
                        LastNav = CurrentTime;

                        // init drive min and max to default values.  We'll reset them to other numbers
                        // if conditions demand it.
                        double driveMax      = 1;
                        double driveMin      = -1;
                        double rampMin       = -0.4;
                        double rampMax       = 0.4;
                        double extensionMin  = -1;
                        double extensionMax  =  1;
                        double feederMin     = -1;
                        double feederMax     =  1;

                        // mapping inputs to motor commands - cube them to desensetize them around
                        // the 0,0 point.  Switching to single stick operation ought to be pretty
                        // straightforward, if that's desired.  Using 2 sticks was simpler to
                        // code up in a hurry.
                        g1_LeftY = g1_LeftY * g1_LeftY * g1_LeftY;
                        g1_RightY = g1_RightY * g1_RightY * g1_RightY;

                        // The ONLY place we set the motor power variables. Set them here, and
                        // we will never have to worry about which set is clobbering the other.
                        // I aligned them this way to make it REALLY clear what's going on.
                        leftDriveCmd = Range.clip(g1_LeftY, driveMin, driveMax);
                        rightDriveCmd = Range.clip(g1_RightY, driveMin, driveMax);

                        //Set loader motors to no power, if either trigger is pressed change power
                        loaderMotorCmd    = 0;
                        if (g2_LT > 0) {
                            loaderMotorCmd = feederMax;
                        }
                        if (g2_RT > 0) {
                            loaderMotorCmd = feederMin;
                        }


                        //Set ramp motord to no power, if either trigger is pressed change power
                       /* rampMotorCmd      = 0;
                        if (g2_A) {
                            rampMotorCmd = rampMax;
                        }
                        if (g2_B) {
                            rampMotorCmd = rampMin;
                        }
                       */
                       //rampMotorCmd = g2_RightY*g2_RightY*g2_RightY;
                        g2_LeftY = g2_LeftY * g2_LeftY * g2_LeftY;
                        g2_RightY = g2_RightY*g2_RightY*g2_RightY;
                        if (Math.abs(g2_RightY) > 0.05){
                            rampMotorCmd = g2_RightY * 0.3;
                        }
                        else {
                            rampMotorCmd = g2_LeftY;
                        }

                        //Only energize the extension motor if the state indicates it is not
                        //deployed
                        extensionMotorCmd = 0;
                        if (rampDeployed == false) {
                            if (g2_RB) {
                                extensionMotorCmd = extensionMax;
                            }
                        }
                        // Backoff the extension motor to reduce tension in frame
                        if ((rampDeployed == true) && (rampBackoff == false)) {
                            extensionCount = extensionCount +1;
                            if (extensionCount > backoffCount) {
                                rampBackoff = true;
                            }
                            extensionMotorCmd = extensionMin;
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
                    if (CurrentTime - LastServo > SERVOPERIOD) {
                        LastServo = CurrentTime;
                        // No servos on the robot
                    }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
                    if (CurrentTime - LastMotor > MOTORPERIOD) {
                        LastMotor = CurrentTime;
                        // Yes, we'll set the power each time, even if it's zero.
                        // this way we don't accidentally leave it somewhere.  Just simpler this way.
                        //robot.leftDrive.setPower(leftDriveCmd);
                        //robot.rightDrive.setPower(rightDriveCmd);

                        // kludge fix for motor mapping
                        //rightDriveCmd = (float)-1*rightDriveCmd;
                        //leftDriveCmd = (float)-1*leftDriveCmd;
                        robot.leftDrive.setPower(rightDriveCmd);
                        robot.rightDrive.setPower(leftDriveCmd);

                        robot.extensionMotor.setPower(extensionMotorCmd);
                        robot.rampMotor.setPower(rampMotorCmd);
                        robot.loaderMotor.setPower(loaderMotorCmd);


                    }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

                    if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                        LastTelemetry = CurrentTime;
                        telemetry.addData("Left Motor Power:       ", leftDriveCmd);
                        telemetry.addData("Right Motor Power:      ", rightDriveCmd);
                        telemetry.addData("Ramp Motor Power:       ", rampMotorCmd);
                        telemetry.addData("Loader Motor Power:     ", loaderMotorCmd);
                        telemetry.addData("Extension Motor Power:  ", extensionMotorCmd);
                        telemetry.addData("Left Trigger  ", g1_LT);
                        telemetry.addData("Right Trigger ", g1_RT);
                        telemetry.addData("Left Bumper   ", g1_LB);
                        telemetry.addData("Right Bumper  ", g1_RB);
                        telemetry.addData("A             ", g1_A);
                        telemetry.addData("B             ", g1_B);
                        telemetry.update();

                        telemetry.update();
                    }


                //telemetry.addData("Left Motor Power:       ", leftDriveCmd);
                //telemetry.addData("Right Motor Power:      ", rightDriveCmd);
                //telemetry.addData("Ramp Motor Power:       ", rampMotorCmd);
                //telemetry.addData("Loader Motor Power:     ", loaderMotorCmd);
                //telemetry.addData("Extension Motor Power:  ", extensionMotorCmd);
                //telemetry.addData("Left Trigger  ", g1_LT);
                //telemetry.addData("Right Trigger ", g1_RT);
                //telemetry.addData("Left Bumper   ", g1_LB);
                //telemetry.addData("Right Bumper  ", g1_RB);
                //telemetry.addData("A             ", g1_A);
                //telemetry.addData("B             ", g1_B);
                //telemetry.update();
            }// end while opmode is active
        //  NEED TO ADD CODE TO FORCE ALL ACTUATORS INTO A SHUTDOWN STATE
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.extensionMotor.setPower(0);
        robot.rampMotor.setPower(0);
        robot.loaderMotor.setPower(0);





    }
}
