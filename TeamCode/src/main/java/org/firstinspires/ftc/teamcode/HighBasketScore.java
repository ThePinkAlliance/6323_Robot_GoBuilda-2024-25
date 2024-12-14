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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
* WARNING:
* At the time of this commit, this was a copied over file from a different repository.
* This file has NOT been testing, or changed yet.
* */


@Autonomous(name="HighBasketScore", group="Auto")

public class HighBasketScore extends LinearOpMode {

    private ElapsedTime     runtime = new ElapsedTime();
    public DcMotor leftDrive = null; //the left drivetrain motor
    public DcMotor rightDrive = null; //the right drivetrain motor
    public DcMotor armMotor = null; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    public DcMotor extendMotor = null;
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     LIFT_SPEED             = 0.7;
    static final double     EXTEND_SPEED           = 0.7;
    static final double     ROTATE_SPEED           = 0.7;
    static final double     MAX_EXTENSION = 45; //max length robot can extend arm to from initial length
    static final double     straightup_angle = 12; //max angle robot can rotate arm to from initial orientation  TODO: adjust this to actual value once known
    static final double     MAX_LIFT = 14; //max distance robot can lift the arm to from initial position TODO: adjust this to actual value once known
    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
//        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive"); // the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); // the right drivetrain motor
        armMotor = hardwareMap.get(DcMotor.class, "arm_lift"); // the arm motor
        extendMotor = hardwareMap.get(DcMotor.class, "arm_extend");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        // Position 0 is the position that the arm starts at.
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos. */
        intake = hardwareMap.get(CRServo.class, "arm_collect");
        wrist = hardwareMap.get(Servo.class, "arm_rotate");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        // Wait for the game to start (driver presses START)
        //Step0: Engage servo and grab sample. (look at mainTeleop for how to do this)
        //Step1: Rotate Arm to 90-degrees
        //Step2: raise arm to Max lift
        //Step3: Extend Arm to max
        //Step4: Move forward 8 to 16 inches (use encoderDrive function to do this)
        //Step5: Rotate arm to 120-degrees position
        //Step6: Release sample
        //Step7: turn robot cw 120-degrees (use encoderDrive function to do this)
        //Step8: Move robot roughly 8 to 10 tiles
        //Step9: DONE! YAAAAAAAAAY!!!!!!!!
        // Wait for the game to start (driver presses START)
        waitForStart();
//        claw_servo.setPosition(45);
        intake.setPower(1);
        sleep(1000);
        intake.setPower(0);
        encoderMove(armMotor,ROTATE_SPEED,straightup_angle,10.0); //rotate arm to straightup angle
        encoderMove(extendMotor, LIFT_SPEED,MAX_LIFT,6.0); //raise arm lift to maxlift
        /*encoderMove(extendArmMotor, EXTEND_SPEED,MAX_EXTENSION, 5.0);*/ //extend arm to max_extension
        encoderDrive(0.6, 14.5, 14.5, 10); // drive forward to basket 15 inches
        //encoderDriveV2(0.3, 15, 0, 0, 15, 10); //moves the robot to the right
        encoderDrive(0.6, 5, -5, 10); // rotate the robot to face the basket
        encoderMove(armMotor,ROTATE_SPEED,3, 10.0); // rotate arm to be over the basket
//        claw_servo.setPosition(0); //drop the sample in the basket
        intake.setPower(-1);
        sleep(500);
        intake.setPower(0);
        encoderMove(armMotor, ROTATE_SPEED,-12, 10.0); //rotate arm away from basket
        encoderDrive(0.6,-6,-6,10.0); //move slightly back from basket before lowering lift
        encoderMove(extendMotor,LIFT_SPEED, -(MAX_LIFT / 2), 10.0); //lower lift to be about half the max height
        encoderDrive(0.6,45,-45,10.0); //rotate 235-degrees toward ascent zone
        encoderDrive(0.6,52,52,10); //drive a little bit
        encoderDrive(0.8,-10,10,10); //turn a bit to touch the bar
        encoderMove(armMotor, ROTATE_SPEED,15, 10.0); //rotate arm so that it touches the bar
        encoderDrive(0.6,6,6,10); //move a little bit if arm did not touch bar
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderMove(DcMotor motor, double speed, double inches, double timeoutS) {
        int target;

        if (opModeIsActive()) {
            target = motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            motor.setTargetPosition(target);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            motor.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() < timeoutS && motor.isBusy()) {
                telemetry.addData("Running to", "%7d", target);
                telemetry.addData("Currently at", "%7d", motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250); // optional pause
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
