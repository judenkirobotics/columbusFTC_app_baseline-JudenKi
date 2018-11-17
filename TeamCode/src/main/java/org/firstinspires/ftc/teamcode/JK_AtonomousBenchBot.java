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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

@Autonomous(name="JK Bench Autonomous", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class JK_AtonomousBenchBot extends LinearOpMode {

    JK_HardwareBenchbot robot   = new JK_HardwareBenchbot();   // Use a Pushbot's hardware
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 10;
    final long MOTORPERIOD = 50;
    //final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;

    float stageTime = 0;
    final int SWG = 0;
    final int LDR = 1;
    final int FWD = 2;
    final int CRB = 3;
    final int WAIT = 4;
    final int RMP = 5;
    int CurrentAutoState = 0;
    int[] stage =          {FWD,   SWG, FWD, SWG,  WAIT};
    long[] f_power =       {20,      0,  10,   0,     0};
    long[] swing_power =   { 0,      5,   0,   5,     0};
    double[] stageLim   =  {25500, 600, 750, 600, 30000};

    int red;
    int green;
    int blue;
    double sPos = 0.0;


    //@Override
    public boolean detectGold(int r, int g, int b){
        boolean gold = false; //default value
        if (g > 0){
            if (((float)(r)/(float)(g) < 2.1) &&
                ((float)(r)/(float)(g) > 1.2) &&
                    ((float)(b)/(float)(g) < 0.9)){
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
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        //long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;


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
   *****************************                CODE          ****************
   ********************
   ************************************************************************************************/


        /* ************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();
  //          telemetry.addData("Current Time: ", CurrentTime);

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
                boolean foundGold = false;
                LastNav = CurrentTime;
                stageTime += NAVPERIOD;
                boolean stageComplete = false;
                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.

                float rampCmd = 0;

                telemetry.addData("Current State: ", CurrentAutoState);
                telemetry.addData("RED         ", red);
                telemetry.addData("GREEN       ", green);
                telemetry.addData("BLUE        ", blue);
                float BG_ratio = (float)blue/(float)green;
                //robot.ColorSensingServo.setPosition(sPos);
                if (detectGold(red, green, blue)){
                    telemetry.addData("GOLD GOLD GOLD!!!!!!", BG_ratio);
                }
                else {
                    //   sPos = sPos + 0.005;
                    telemetry.addData("NOT GOLD",BG_ratio);
                }

                switch (stage[CurrentAutoState]) {
                    //        int[] stage =          {BAL,FWD, CRB, FWD, RMP, FWD};
                    case SWG:
                        telemetry.addData("RED         ", red);
                        telemetry.addData("GREEN       ", green);
                        telemetry.addData("BLUE        ", blue);
                        robot.ColorSensingServo.setPosition(sPos);
                        if (detectGold(red, green, blue)){
                            telemetry.addData("GOLD GOLD GOLD!!!!!!", red);
                        }
                        else {
                         //   sPos = sPos + 0.005;
                            telemetry.addData("NOT GOLD",red);
                        }



                        break;
                    case FWD:

                        break;
                    case CRB:
                        break;
                    case LDR:
                        break;
                    case RMP:
                        break;
                    case WAIT:
                        break;
                    default:
                        break;
                }
                telemetry.addData("RMPcmdbeforereset", rampCmd);
                telemetry.update();
                if (stageTime > stageLim[CurrentAutoState]) {
                    stageTime = 0;
                    CurrentAutoState++;
                    CurrentAutoState = Range.clip(CurrentAutoState,0,WAIT);
                    telemetry.addData("Current State: ", CurrentAutoState);
                }
                // mapping inputs to servo command


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


            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/





            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.update();
            }
        } // end of while opmode is active

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}