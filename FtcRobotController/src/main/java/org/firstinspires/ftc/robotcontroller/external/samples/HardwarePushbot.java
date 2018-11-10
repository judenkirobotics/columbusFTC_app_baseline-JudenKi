/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    //public DcMotor relicExtendMotor = null;
    //public Servo   relicPivotServo  = null;
    //public Servo   relicClawServo   = null;
    /* Public OpMode members. */
    public DcMotor     leftDrive      = null;
    public DcMotor     rightDrive     = null;
    public DcMotor     leftRear      = null;
    public DcMotor     rightRear     = null;
    public DcMotor     rampMotor      = null;
    public DcMotor     extensionMotor = null;
    public DcMotor     loaderMotor    = null;

    //public GyroSensor  gyro           = null;
    //public TouchSensor extensionTouch = null;  // Ramp Deployment
    //public AnalogOutput leftLed       = null;
    //public AnalogOutput rightLed       = null;
   // public OpticalDistanceSensor Odsleft = null;
   // public OpticalDistanceSensor Odsright = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        /*BLAH test
        relicExtendMotor= hwMap.get(DcMotor.class, "relicExtendMotor");
        relicClawServo=hwMap.get(Servo.class,"relicClawServo");
        relicPivotServo=hwMap.get(Servo.class,"relicPivotServo");
        */
        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        //rampMotor  = hwMap.get(DcMotor.class, "beltmotor");
        //loaderMotor= hwMap.get(DcMotor.class, "loadermotor");
        //extensionMotor = hwMap.get(DcMotor.class, "pullymotor");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
      //  Odsleft = hwMap.get(OpticalDistanceSensor.class, "left_dist");
       // Odsright = hwMap.get(OpticalDistanceSensor.class, "right_dist");

        //    public OpticalDistanceSensor Odsleft = null;
        //ODS code written by Tarun and John


        //leftLed = hwMap.get(AnalogOutput.class, "leftLed");
        //rightLed = hwMap.get(AnalogOutput.class, "rightLed");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //rampMotor.setDirection(DcMotor.Direction.FORWARD);
        //loaderMotor.setDirection(DcMotor.Direction.FORWARD);
        //extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //rampMotor.setPower(0);
        //loaderMotor.setPower(0);
        //extensionMotor.setPower(0);
        /*blah test
        relicClawServo.setDirection(Servo.Direction.FORWARD);
        relicClawServo.setPosition(0);
        relicPivotServo.setDirection(Servo.Direction.FORWARD);
        relicPivotServo.setPosition(0);
        relicExtendMotor.setDirection(DcMotor.Direction.FORWARD);
        relicExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicExtendMotor.setPower(0);
        */
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //loaderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //Define and Initialize Sensors
        //gyro = hwMap.get(GyroSensor.class, "gyro");
        //extensionTouch = hwMap.get(TouchSensor.class, "ext_touch");
    }
 }

