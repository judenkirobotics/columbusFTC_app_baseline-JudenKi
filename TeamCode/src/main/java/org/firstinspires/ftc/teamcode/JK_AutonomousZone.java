package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 *
 * Concept by Howard
 *
 * First Coding by Jeffrey and Alexis
 *
 *
 *
 */

//0, 50, 0, 50, 0

/*
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
*/
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
/*Made by Tarun and John*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

@Autonomous(name="JK Autonomous Zone Side", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class JK_AutonomousZone extends LinearOpMode {

    JK_19_HardwarePushbot robot   = new JK_19_HardwarePushbot();   // Use a Pushbot's hardware
    final long SENSORPERIOD = 50;
    //final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 10;
    final long MOTORPERIOD = 50;
    //final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;
    final double MAX_S_POS = 0.85;

    boolean gold_detected = false;
    boolean third_position = true;

    float stageTime = 0;
    final int SWG = 0;
    final int FWD = 1;
    final int KIK = 2;
    final int WAIT = 3;
    final float MAX_RED_GREEN = (float)2.1;
    final float MIN_RED_GREEN = (float)0.95;
    final float MAX_BLUE_GREEN = (float)0.9;
    final float MIN_BLUE_GREEN = (float)0.0;
    final double SWING_HOME = 0.1;
    final double SWING_TRANSIT = 0.4;
    final long KIK_TRANSIT_TIME = 250;
    final long SWING_TRANSIT_TIME = KIK_TRANSIT_TIME + 200;

    int CurrentAutoState = 0;
    int[] stage =       {FWD,  SWG, FWD,  FWD,   KIK, WAIT};
    double[] l_power =  {-0.5,   0,-0.5, -0.3,     0,    0};
    double[] r_power =  {-0.5,   0, 0.5, -0.3,     0,    0};
    long[] paddle    =  { 0,     0,   0,    1,     1,    0};
    //double[]paddleTime = { 0,    0,   0,    0,  1000,    0};
    double[] stageLim = {470, 4500, 740, 1200,  1200, 1000};

    int red;
    int green;
    int blue;
    double sPos = 0.0;
    double pPower = 0.0;
    double leftMotorCmd = 0.0;
    double rightMotorCmd = 0.0;

    //@Override
    public boolean detectGold(int r, int g, int b){
        boolean gold = false; //default value
        if (g > 0){
            if (((float)(r)/(float)(g) < MAX_RED_GREEN) &&
                ((float)(r)/(float)(g) > MIN_RED_GREEN) &&
                ((float)(b)/(float)(g) < MAX_BLUE_GREEN)){
                gold = true; // changes to true only if the if statement is satisfied
            }
        }
        return (gold);
    }
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastTelemetry = CurrentTime + 17;

        sPos = robot.ColorSensingServo.getPosition();  // Set initial value

        telemetry.addData("Status ", "Initialized");
        telemetry.addData("SPos ", sPos);
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //A Timing System By Katherine Jeffrey,and Alexis
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
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
                red = robot.MineralColorSensor.red();
                green = robot.MineralColorSensor.green();
                blue = robot.MineralColorSensor.blue();
            }

        /*  ***********************************************************************
         ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
        /* vvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/

            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;
                stageTime += NAVPERIOD;
                telemetry.addData("Current State: ", CurrentAutoState);
                telemetry.addData("sPos        ", sPos);
                switch (stage[CurrentAutoState]) {
                    case FWD:
                        if ((stageTime >= stageLim[CurrentAutoState]) ||
                                (gold_detected)){
                            leftMotorCmd = 0;
                            rightMotorCmd = 0;
                            pPower = 0;
                            stageTime = 0;
                            CurrentAutoState++;
                        }
                        else if (!gold_detected){
                            if (third_position) {
                                if (stageTime > SWING_TRANSIT_TIME) {
                                    sPos = SWING_TRANSIT;
                                }
                                if (stageTime > KIK_TRANSIT_TIME) {
                                    pPower = paddle[CurrentAutoState];
                                }
                            }
                            leftMotorCmd = l_power[CurrentAutoState];
                            rightMotorCmd = r_power[CurrentAutoState];
                        }

                        break;
                    // Swing arm looking for gold block, if found knock it off
                    case SWG:
                        if ((stageTime > stageLim[CurrentAutoState]) ||
                            (gold_detected) ||
                            (sPos >= MAX_S_POS)) {
                            stageTime = 0;
                            CurrentAutoState++;
                        }
                        else {
                            gold_detected = gold_detected || detectGold(red, green, blue);
                            sPos = sPos + 0.003;
                        }
                        break;
                    case KIK:
                        if ((stageTime >= stageLim[CurrentAutoState]) ||
                            (!gold_detected)){
                            stageTime = 0;
                            pPower = 0;
                            CurrentAutoState++;
                        }
                        else {
                            pPower = paddle[CurrentAutoState];
                        }
                        break;

                    case WAIT:
                        pPower = 0;
                        leftMotorCmd = 0.0;
                        rightMotorCmd = 0.0;
                        telemetry.addData("Waiting... ", red);
                        break;
                    default:
                        break;
                }
                telemetry.update();
                if (stageTime > stageLim[CurrentAutoState]) {
                    stageTime = 0;
                    CurrentAutoState++;
                    CurrentAutoState = Range.clip(CurrentAutoState,0,WAIT);
                    telemetry.addData("Current State: ", CurrentAutoState);
                }
            }
            // END NAVIGATION

        /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^*/
        /* ********************************************************************************/
        /*  ***********************************************************************
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
         *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/

            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;

                robot.leftDrive.setPower(leftMotorCmd);
                robot.rightDrive.setPower(rightMotorCmd);

            }

            /* ***************************************************
             *                SERVO OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the servos
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
                robot.ColorSensingServo.setPosition(sPos);
                robot.PaddleServo.setPower(pPower);
            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                telemetry.addData("RED: ", red);
                telemetry.addData("GREEN: ", green);
                telemetry.addData("BLUE: ", blue);
                LastTelemetry = CurrentTime;
                telemetry.update();
            }
        } // end of while opmode is active

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}