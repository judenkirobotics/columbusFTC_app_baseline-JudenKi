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
    DcMotor[] leftMotors = new DcMotor[]{robot.leftDrive};
    DcMotor[] rightMotors = new DcMotor[]{robot.rightDrive};
    Drive myDrive = new Drive(leftMotors, rightMotors);
    TouchSensor touchSensor;
    GyroSensor myGyro;
    //touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
    //   private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    //   private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    //   private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //   private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //           (WHEEL_DIAMETER_INCHES * 3.1415);
    //   private static final double DRIVE_SPEED = 0.6;
    //   private static final double TURN_SPEED = 0.5;


    /* Public OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor riser = null;

    // Define class members
    public Servo leftClamp = null;
    public Servo rightClamp = null;

    //  static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //  static final int CYCLE_MS = 50;     // period of each cycle
    //  static final double MAX_POS = 1.0;     // Maximum rotational position
//    static final double MIN_POS = 0.0;     // Minimum rotational position
    static final double LEFTCLAMPED = 45;
    static final double LEFTUNCLAMPED = -5;
    static final double RIGHTCLAMPED = 5;
    static final double RIGHTUNCLAMPED = -45;

    static final double CLAMP_MOTION_TIME = 250;

    //double clampOffset = 0;                       // Servo mid position
    //final double CLAMP_SPEED = 0.02;                   // sets rate to move servo
    final int  AUTO_STATES = 4;
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;

    final long MAX_MOVE_TIME = 4000;

    final double PROPGAIN = 0.6;
    final double INTGAIN  = 0.3;
    final double DERGAIN  = 0.1;
    final long PIDMAXDUR  = 3;

    final int RISER_DISTANCE = 500;
    int CurrentAutoState = 0;
    int rightMotorPos;
    int lefMotorPos;
    int riserMotorPos;
    int prevRiserErr = 0;
    int rampStressCtr = 0;
    int currentHeading = 0;

    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    public double simplePID (double err, double duration, double prevErr)
    {
        double pidmin = -.7;
        double pidmax = 0.7;
        double propTerm = Range.clip(PROPGAIN * err, pidmin, pidmax);
        double intTerm = Range.clip(duration/PIDMAXDUR,pidmin,pidmax)*INTGAIN;
        double derTerm = Range.clip((err - prevErr),pidmin,pidmax) * DERGAIN;
        return (Range.clip(propTerm + intTerm + derTerm, pidmin,pidmax));
    }
    public boolean detectItem() {
        // presuming there will be a detect item here ... populate this code when we know that
        return true;
    }
    public boolean moveLever() {
        // presuming we will move a lever somehow.  Populate this method when that is known.
        return true;
    }
    float extendRamp(boolean rampDeployed) {
        float extensionMin  = (float)-0.7;
        float extensionMax  =  (float)0.7;
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
    float forwardMove(int cmdDist, float cmdPwr )
    {
        float driveMotorCmd = 0;

        return driveMotorCmd;
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
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        long liftDuration = 0;
        long liftOffDuration = 0;

        // variables for controller inputs.
        float g1_leftX;
        float g1_LeftY;
        float g1_RightX;
        float g1_RightY;
        int g1_A_Counts = 0;

        //double leftClamp_Cmd = LEFTUNCLAMPED;
        //double rightClamp_Cmd = RIGHTUNCLAMPED;
        double loaderMotorCmd     = 0;
        double rampMotorCmd = 0;

        float stageTime = 0;
        int startHeading = 0;
        double startPos = 0;
        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float extensionMotorCmd = 0;
        float riserCmd = 0;
        /*{ , , , , ,           0   1   2    3   4  5  6  7  8  9  10*/
        int[] TurnArray =      {0, 45,  0,   2,  0, 0, 0, 0, 0, 0,  0};
        int[] TurnPower =      {0, 40,  0, -30,  0, 0, 0, 0, 0, 0,  0};
        float[] StraightPwr=   {0, 0,  30,   0,  0, 0, 0, 0, 0, 0,  0};
        float[] LoaderPwr =    {0, 0,   0,   1,  0, 0, 0, 0, 0, 0,  0};
        float[] rampMotorPwr = {0, 0,   0,   1,  0, 0, 0, 0, 0, 0,  0};
        int[] StraightDist=    {0, 0, 50,    0,  0, 0, 0, 0, 0, 0,  0};
        boolean rampLimitReached = false;
        double extensionSwitchTime = 0;

        boolean g1_A;
        //boolean g1_B;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //
        int riserZero = riser.getCurrentPosition();

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

            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                //gyro
                currentHeading = myGyro.getHeading();

                // probably more "pure" just to read the touch sensor here
                // and interpret the results in NAV.  We can probably deal with the
                // minor abuse to both cohesion and decomposition with no really bad
                // side effects, however.
                rampLimitReached = (robot.extensionTouch.isPressed()|| rampLimitReached);
                telemetry.addData("Ramp Extension", "Is Not Pressed");
            }


            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.
                rightMotorPos = robot.rightDrive.getCurrentPosition();
                lefMotorPos = robot.leftDrive.getCurrentPosition();

            }
             /*         NO CONTROLLER INPUT FOR AUTONOMOUS       *
             ********* THIS COMMENT IS JUST A REFERENCE**********/



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
                int extensionMaxTime = 1350;  //milliseconds

                switch ( CurrentAutoState ) {
                    case 0:
                        extensionMotorCmd = extendRamp(rampLimitReached);

                        // extens..Time != stageTime. It tells how long the ramp was extended. Persistent
                        extensionSwitchTime += NAVPERIOD;

                        if ((extensionSwitchTime > extensionMaxTime) ||
                            (extensionMotorCmd == 0 )) {
                            stageComplete = true;
                        }
                        break;
                    case 1: case 3:
                        /* gyroturn5
                         *  startHeading  - input. heading when this "state" started
                         *  currHeading   - input. what is the heading when gt5 invoked
                         *  newHeading    - input. Destination heading
                         *  turnPwr       - input. -100 to 100, percent power. negative means counterclockwise
                         *  turnTime      - input. how long this "state" has been in play
                         * @return pwrSet - multiply the pwrSet by -1 for the starboard motor in the calling routine.
                          */
                        int target = TurnArray[CurrentAutoState];
                        int turnvec = TurnPower[CurrentAutoState];
                        rightDriveCmd = myDrive.gyroTurn5(startHeading,currentHeading,target,turnvec,stageTime);
                        leftDriveCmd = (float)-1*rightDriveCmd;

                        if ((rightDriveCmd == 0) || (stageTime > MAX_MOVE_TIME)) {
                            rightDriveCmd = 0;
                            leftDriveCmd = 0;
                            stageComplete = true;
                        }
                        break;
                    case 2:
                        //
                        float motorCommands = forwardMove( StraightDist[CurrentAutoState],StraightPwr[CurrentAutoState]);
                        if ((stageTime >= 3000) ||
                            (motorCommands == 0))
                        {
                            stageComplete = true;
                            leftDriveCmd = motorCommands;
                            rightDriveCmd = motorCommands;
                        }
                        break;
                    case 4: //Pick up the block: Run loader and ramp for <2 second only pick 1 block
                        if (stageTime < 2000) {
                            loaderMotorCmd = LoaderPwr[CurrentAutoState];
                            rampMotorCmd = rampMotorPwr[CurrentAutoState];

                        }else{
                            loaderMotorCmd = 0;
                            rampMotorCmd = 0;
                            stageComplete = true;
                        }

                        break;
                    case 5:
                        stageComplete = moveLever();
                        break;
                }
                if (stageComplete) {
                    //startPos = currentPos;
                    startHeading = currentHeading;
                    //startPos = 0;
                    //startHeading = 0;
                    stageTime = 0;
                    CurrentAutoState++;
                }
                // mapping inputs to servo command

                // The ONLY place we set the motor power request. Set them here, and
                // we will never have to worry about which set is clobbering the other.

                // motor commands: Clipped & clamped.
                leftDriveCmd      = Range.clip(leftDriveCmd,  driveMin, driveMax);
                rightDriveCmd     = Range.clip(rightDriveCmd, driveMin, driveMax);
                extensionMotorCmd = Range.clip(rampCmd,       riserMin, riserMax);
                rampMotorCmd      = Range.clip(rampMotorCmd,  rampMin,  rampMax);
                loaderMotorCmd    = Range.clip(loaderMotorCmd,loaderMin,loaderMax);
            }
            // END NAVIGATION


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

                // Move both servos to new position.
                //leftClamp.setPosition(leftClamp_Cmd);
                //rightClamp.setPosition(rightClamp_Cmd);
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
                /*  Left Drive Motor Power  */

                // kludge fix for motor mapping
                //rightDriveCmd = (float)-1*rightDriveCmd;
                //leftDriveCmd = (float)-1*leftDriveCmd;
                robot.leftDrive.setPower(rightDriveCmd);
                robot.rightDrive.setPower(leftDriveCmd);

                /* Loader Motor Power */
                robot.loaderMotor.setPower(loaderMotorCmd);

                robot.rampMotor.setPower(rampMotorCmd);
                /* Lifter Motor Power   */
                robot.extensionMotor.setPower(extensionMotorCmd);
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
        robot.extensionMotor.setPower(0);
        robot.loaderMotor.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}