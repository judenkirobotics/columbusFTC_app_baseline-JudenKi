package org.firstinspires.ftc.teamcode;

/**
 * Created by howard on 12/2/17.
 * based on Drive, created by JudenKi (Jim) on 11/12/16
 */


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;


public class Drive2 {
    public static int prevHeading = 176;
    public static int newHeading = 178;
    private DcMotor[] leftMotors   = null;
    private DcMotor[] rightMotors  = null;

    private double MIN_DRIVE_DISTANCE = 0.0;
    private double MAX_DRIVE_DISTANCE = 120.0;

    private double FAST_POWER          = 0.6;
    private double SLOW_POWER          = 0.3;
    private double CORRECTION_POWER    = 0.15;
    private double RIGHT_SIGN          = 1;
    private double LEFT_SIGN           =-1;
    public static boolean RIGHT_TURN   = true;
    public static boolean LEFT_TURN    = false;
    private GyroSensor driveGyro       = null;
    private LinearOpMode myMode        = null;
    public static final float MAX_TURN_TIME = (float)4000;
    public static final float BUMP_TIME = (float)2000;

    private double WHEEL_CIRC    = 13;
    private double WHEEL_RPM     = 2;   //Misnamed, fix should be RPS second not minute
    private double TURN_PER_SECOND = 79.5;
    private long driveStopTime  = 0;
    private boolean motorsStopped = true;

    // Left and right are with respect to the robot
    public Drive2( DcMotor[] _leftMotors, DcMotor[] _rightMotors ) {
        assert leftMotors != null;
        assert rightMotors != null;
        assert leftMotors.length > 0;
        assert rightMotors.length > 0;
        this.leftMotors = _leftMotors;
        this.rightMotors = _rightMotors;
        // Set all DC Motors to run without encoders
        for( DcMotor dcm : leftMotors ) {
            dcm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //dcm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //dcm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    public void drift(double power) {
        if (power > 0) {
            for(DcMotor dcm : leftMotors){
                dcm.setPower(dcm.getPower() +(power));
            }
            for(DcMotor dcm : rightMotors) {
                dcm.setPower(dcm.getPower() +(power));
            }
        }
        else {
            for(DcMotor dcm : leftMotors){
                dcm.setPower(dcm.getPower() +(power));
            }
            for(DcMotor dcm : rightMotors) {
                dcm.setPower(dcm.getPower() +(power));
            }
        }
    }

    public void driveMove(double forwardPower, double driftPower){
        for (DcMotor dcm : leftMotors){
            dcm.setPower( - forwardPower + driftPower);
        }
        for (DcMotor dcm : rightMotors){
            dcm.setPower( forwardPower + driftPower);
        }
    }


    public float moveForward2(double startPos, double distance , float power) {

        double moveDistance;
        long   time;

        // Input distance will be in inches, perform a range check
        moveDistance = rangeCheck(distance);

        //Calculate how long to move.
        time =  moveTime(moveDistance, WHEEL_RPM, WHEEL_CIRC);

        driveStopTime = time + SystemClock.elapsedRealtime();
        // once we figure out the encoders, we'll put that in here.
        return power;
    }

    /***********************************************************
     * gyroturn5
     *  startHeading  - input. heading when this "state" started
     *  currHeading   - input. what is the heading when gt5 invoked
     *  newHeading    - input. Destination heading
     *  turnPwr       - input. -100 to 100, percent power. negative means counterclockwise
     *  turnTime      - input. how long this "state" has been in play
     * @return pwrSet - multiply the pwrSet by -1 for the starboard motor in the calling routine.
     *   Note: The gyroturn5 will quit after MAX_TURN_TIME, and will give a "bump" to increase turn
     *   power after BUMP_TIME.  The idea is to juice the power
     */
    public static float gyroturn5(int startHeading, int currHeading, int newHeading, int turnPwr, float turnTime){
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



    public void update() {
        if  (SystemClock.elapsedRealtime()>driveStopTime) {
            for(DcMotor dcm : leftMotors) {
                dcm.setPower(0.0);
            }
            for(DcMotor dcm : rightMotors) {
                dcm.setPower(0.0);
            }
            motorsStopped = true;
        }
    }


    public boolean motorsRunning () {
        return (!motorsStopped);
    }

    public void allStop(){
        for(DcMotor dcm : leftMotors) {
            dcm.setPower(0.0);
        }
        for(DcMotor dcm : rightMotors) {
            dcm.setPower(0.0);
        }
        motorsStopped = true;
    }

    public void setParams(double wheelCirc, double wheelRPM, double turnPerSecond,
                          double fastPower, double slowPower, double correctionPower,
                          double rightSign, double leftSign, GyroSensor gyro,
                          LinearOpMode operationMode) {
        WHEEL_CIRC = wheelCirc;
        WHEEL_RPM = wheelRPM;
        TURN_PER_SECOND = turnPerSecond;
        FAST_POWER          = fastPower;
        SLOW_POWER          = slowPower;
        CORRECTION_POWER    = correctionPower;
        RIGHT_SIGN          = rightSign;
        LEFT_SIGN           = leftSign;
        driveGyro           = gyro;
        myMode = operationMode;


    }


    private double rangeCheck(double distance) {
        if (distance < MIN_DRIVE_DISTANCE)
            return 0;
        if (distance > MAX_DRIVE_DISTANCE)
            return MAX_DRIVE_DISTANCE;
        return distance;
    }

    private long moveTime(double distance, double rpm, double circumferance) {
        return (long)(distance / (rpm * circumferance) * 1000);
    }

    private long turnTime(double degrees) {
        return (long)((degrees / TURN_PER_SECOND) *1000);
    }



}