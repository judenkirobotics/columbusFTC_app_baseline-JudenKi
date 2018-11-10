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
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

@Autonomous(name="JK Red Left Autonomous", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class JK_AtonomousRedLeft extends LinearOpMode {
    //Encoder enc;
    //enc = new Encoder(0,1,false,Encoder.EncodingType.k4X);
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    //DcMotor[] leftMotors = new DcMotor[]{robot.left_Drive};
    //DcMotor[] rightMotors = new DcMotor[]{robot.right_Drive};
    //Drive myDrive = new Drive(left_Drive, rightMotors);
    //TouchSensor touchSensor;
    //GyroSensor myGyro;
    //touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
    //   private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    //   private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    //   private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //   private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //           (WHEEL_DIAMETER_INCHES * 3.1415);
    //   private static final double DRIVE_SPEED = 0.6;
    //   private static final double TURN_SPEED = 0.5;


    /* Public OpMode members. */
    //private DcMotor left_Drive = null;
    //private DcMotor right_Drive = null;
    //private DcMotor riser = null;

    // Define class members
    //public Servo leftClamp = null;
    //public Servo rightClamp = null;

    //  static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //  static final int CYCLE_MS = 50;     // period of each cycle
    //  static final double MAX_POS = 1.0;     // Maximum rotational position
//    static final double MIN_POS = 0.0;     // Minimum rotational position
    //static final double LEFTCLAMPED = 45;
    //static final double LEFTUNCLAMPED = -5;
    //static final double RIGHTCLAMPED = 5;
    //static final double RIGHTUNCLAMPED = -45;

    //static final double CLAMP_MOTION_TIME = 250;

    //double clampOffset = 0;                       // Servo mid position
    //final double CLAMP_SPEED = 0.02;                   // sets rate to move servo
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 10;
    final long MOTORPERIOD = 50;
    //final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;
    public static final float MAX_TURN_TIME = (float)4000;
    public static final float BUMP_TIME     = (float)2000;

    final long MAX_MOVE_TIME = 4000;

    //final double PROPGAIN = 0.6;
    //final double INTGAIN  = 0.3;
    //final double DERGAIN  = 0.1;
    //final long PIDMAXDUR  = 3;

    int CurrentAutoState = 0;
    int rightMotorPos;
    int leftMotorPos;
    int rampStressCtr = 0;
    int currentHeading = 0;

    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    /*public double simplePID (double err, double duration, double prevErr)
    {
        double pidmin = -.7;
        double pidmax = 0.7;
        if (Math.abs(err) < 10){
            err = 0;
            duration = 0;
            prevErr = 0;
        }
        double propTerm = Range.clip(PROPGAIN * err, pidmin, pidmax);
        double intTerm = Range.clip(duration/PIDMAXDUR,pidmin,pidmax)*INTGAIN;
        double derTerm = Range.clip((err - prevErr),pidmin,pidmax) * DERGAIN;
        return (Range.clip(propTerm + intTerm + derTerm, pidmin,pidmax));
    }*/
    //public boolean detectItem() {
        // presuming there will be a detect item here ... populate this code when we know that
    //    return true;
    //}
    //public boolean moveLever() {
        // presuming we will move a lever somehow.  Populate this method when that is known.
    //    return true;
    //}
    public float gyroTurn5(int startHeading, int currHeading, int newHeading, int turnPwr, float turnTime){
        float pwrSet = turnPwr;
        int accumTurn = Math.abs(startHeading - currHeading);
        accumTurn = (accumTurn > 360)? (360-accumTurn):accumTurn;
        int cw = (turnPwr < 0)? -1: 1;
        int transit = (((currHeading > newHeading) && (cw > 0)) ||
                ((currHeading < newHeading) && (cw < 0))) ?  360 : 0;
        int desiredRotation = Math.abs(transit + (cw*newHeading) + ((-1*cw)*currHeading));
        desiredRotation = (desiredRotation > 360) ? desiredRotation - 360 : desiredRotation;
        if((accumTurn < desiredRotation) && (turnTime < MAX_TURN_TIME)){
            pwrSet = (turnTime > BUMP_TIME)? (float)turnPwr: (float)1.0;
        }
        else
        {
            pwrSet = 0;
        }
        return pwrSet;
    }
    public float Fwd5(double traveled,double goal,float pwr, double duration)
    {
        float multiplier = (duration > 2000)? (float)1.2:(float)1.0;

        float cmdPwr = 0;
        if (traveled < goal) cmdPwr = -1*pwr*multiplier/(float)100.0;
        cmdPwr = Range.clip(cmdPwr, -1, 1);
        return cmdPwr;
    }

    float extendRamp(boolean rampDeployed) {
        float extensionMin  = (float)-1;
        float extensionMax  =  (float)1;
        float extensionMotorCmd = 0;
        if (!rampDeployed) {
            extensionMotorCmd = extensionMax;
        }
        else if (rampStressCtr <= 4)
        {
            rampStressCtr++;
            extensionMotorCmd = extensionMin;
        }
        return extensionMotorCmd;
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

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        //long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

/*        long liftDuration = 0;
        long liftOffDuration = 0;

        // variables for controller inputs.
        float g1_leftX;
        float g1_LeftY;
        float g1_RightX;
        float g1_RightY;
        int g1_A_Counts = 0;*/

        //double leftClamp_Cmd = LEFTUNCLAMPED;
        //double rightClamp_Cmd = RIGHTUNCLAMPED;
        double loaderMotorCmd   = 0;
        double rampMotorCmd     = 0;

        float stageTime = 0;
        int startHeading = 0;
        double startLeft = 0;
        double startRight = 0;
        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float leftRearCmd = 0;
        float rightRearCmd = 0;
        float extensionMotorCmd = 0;
        final int BAL = 0;
        final int LDR = 1;
        final int FWD = 2;
        final int CRB = 3;
        final int WAIT = 4;
        final int RMP = 5;
        /*{ , , , , ,           0    1    2    3     4    5    6  */
        int[] stage =          {BAL, FWD, CRB, FWD,  RMP, LDR, FWD, WAIT};
        int[] TurnArray =      {  0,   0,   0,   0,    0,   0,   0,     0};
        int[] TurnPower =      {  0,   0,   0,   0,    0,   0,   0,     0};
        float[] StraightPwr=   {  0, -50,   0, -50,    0,   0,  50,     0};
        double[] stageLim   =  {500, 600, 750, 600,   30, 200, 300, 25000};
        float[] crabPwr =      {  0,   0,  60,   0,    0,   0,   0,     0};
        boolean rampLimitReached = false;
        double extensionSwitchTime = 0;

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
            telemetry.addData("Current Time: ", CurrentTime);

            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                //gyro
                //currentHeading = myGyro.getHeading();

                //rampLimitReached = (robot.extensionTouch.isPressed()|| rampLimitReached);
                //telemetry.addData("Ramp Extension", "Is Not Pressed");
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
                boolean stageComplete = false;
                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.
                float driveMax = 1;
                float driveMin = -1;
                float riserMax = 1;
                float riserMin = -1;
                float rampMin = -1;
                float rampMax = 1;
                float loaderMin = -1;
                float loaderMax = 1;

                float rampCmd = 0;
                int extensionMaxTime = 1750;  //milliseconds
                telemetry.addData("Current State: ", CurrentAutoState);

                switch (stage[CurrentAutoState]) {
                    //        int[] stage =          {BAL,FWD, CRB, FWD, RMP, FWD};
                    case BAL:
                        break;
                    case FWD:
                        leftDriveCmd = StraightPwr[CurrentAutoState]/(float)100;
                        rightDriveCmd = StraightPwr[CurrentAutoState]/(float)100;
                        leftRearCmd = StraightPwr[CurrentAutoState]/(float)100;
                        rightRearCmd = StraightPwr[CurrentAutoState]/(float)100;
                        break;
                    case CRB:
                        /*
                        leftDriveCmd = crabPwr[CurrentAutoState]/(float)-100;
                        leftRearCmd = crabPwr[CurrentAutoState]/(float)100;
                        rightDriveCmd = crabPwr[CurrentAutoState]/(float)100;
                        rightRearCmd = crabPwr[CurrentAutoState]/(float)-100;
                        */
                        break;
                    case LDR:
                        loaderMotorCmd = 1;
                        break;
                    case RMP:
                        rampCmd = 2/(float)5;
                        telemetry.addData("TimebeforeMeasure", CurrentTime - LastNav);
                        telemetry.update();
                        break;
                    case WAIT:
                        leftDriveCmd      = 0;
                        rightDriveCmd     = 0;
                        leftRearCmd       = 0;
                        rightRearCmd      = 0;
                        loaderMotorCmd    = 0;
                        rampCmd           = 0;
                        break;
                    default:
                        //stageComplete = true;
                        // we've arrived here - either we're done or something has gone badly wrong
                        leftDriveCmd      = 0;
                        rightDriveCmd     = 0;
                        leftRearCmd       = 0;
                        rightRearCmd      = 0;
                        loaderMotorCmd    = 0;
                        rampCmd           = 0;
                        break;
                }
                telemetry.addData("RMPcmdbeforereset", rampCmd);
                telemetry.update();
                if (stageTime > stageLim[CurrentAutoState]) {
                    leftDriveCmd      = 0;
                    leftRearCmd      = 0;
                    rightDriveCmd     = 0;
                    rightRearCmd     = 0;
                    loaderMotorCmd    = 0;
                    rampCmd           = 0;
                    stageTime = 0;
                    CurrentAutoState++;
                    CurrentAutoState = Range.clip(CurrentAutoState,0,WAIT);
                    telemetry.addData("Current State: ", CurrentAutoState);
                }
                // mapping inputs to servo command

                // The ONLY place we set the motor power request. Set them here, and
                // we will never have to worry about which set is clobbering the other.

                // motor commands: Clipped & clamped.
                leftDriveCmd      = Range.clip(leftDriveCmd  ,driveMin, driveMax);
                rightDriveCmd     = Range.clip(rightDriveCmd ,driveMin, driveMax);
                leftRearCmd       = Range.clip(leftRearCmd   ,driveMin, driveMax);
                rightRearCmd      = Range.clip(rightRearCmd  ,driveMin, driveMax);
                rampMotorCmd      = Range.clip(rampCmd       ,riserMin, riserMax);
                loaderMotorCmd    = Range.clip(loaderMotorCmd,loaderMin,loaderMax);
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

                robot.leftDrive.setPower(rightDriveCmd);
                robot.rightDrive.setPower(leftDriveCmd);
                robot.leftRear.setPower(leftRearCmd);
                robot.rightRear.setPower(rightRearCmd);

                /* Loader Motor Power */
                robot.loaderMotor.setPower(loaderMotorCmd);
                robot.rampMotor.setPower(rampMotorCmd);


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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.rampMotor.setPower(0);
        robot.loaderMotor.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}