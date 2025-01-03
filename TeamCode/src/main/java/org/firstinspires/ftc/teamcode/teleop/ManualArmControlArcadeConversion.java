/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Arm;
import org.firstinspires.ftc.teamcode.lib.Drivetrain;


@TeleOp(name = "Teleop: Manual Arm Control (Using Arm and DriveTrain class)", group = "Teleop")
public class ManualArmControlArcadeConversion extends OpMode {
    Arm arm;
    Drivetrain driveTrain;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    @Override
    public void init() {
        this.arm = new Arm(hardwareMap);
        this.driveTrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1;
        double rotate = gamepad1.right_stick_x * -1;

        double left = (forward + rotate * 1.5) / 2;
        double right = (forward - rotate * 1.5) / 2;

        int armLastPosition = 0; // This is used in the if-else idle range statement, along with wasUsingPower
        boolean wasUsingPower = true;

        driveTrain.set_power(left, right);
        arm.set_extend_power(gamepad1.right_stick_y);

        if (gamepad2.a) {
            arm.set_collect_power(INTAKE_COLLECT);
        } else if (gamepad2.x) {
            arm.set_collect_power(INTAKE_OFF);
        } else if (gamepad2.b) {
            arm.set_collect_power(INTAKE_DEPOSIT);
        }

        if (gamepad2.left_stick_y >= -0.1 && gamepad2.left_stick_y <= 0.1) {
            if (wasUsingPower) {
                armLastPosition = arm.get_lift_position();
                wasUsingPower = false;
            }

            arm.set_lift_position(armLastPosition);
            arm.set_arm_mode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            wasUsingPower = true;
            arm.set_lift_power(gamepad2.left_stick_y);
            arm.set_arm_mode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /* Run until the driver presses stop */
//        while (opModeIsActive()) {
//
//            //set power to extendMotor to the value of gamepad2 right_stick_y
//            extendMotor.setPower(gamepad2.right_stick_y);
//            /* Here we "mix" the input channels together to find the power to apply to each motor.
//            The both motors need to be set to a mix of how much you're retesting the robot move
//            forward, and how much you're requesting the robot turn. When you ask the robot to rotate
//            the right and left motors need to move in opposite directions. So we will add rotate to
//            forward for the left motor, and subtract rotate from forward for the right motor. */
//
//            left = forward + rotate;
//            right = forward - rotate;
//
//            /* Normalize the values so neither exceed +/- 1.0 */
//            max = Math.max(Math.abs(left), Math.abs(right));
//            if (max > 1.0) {
//                left /= max;
//                right /= max;
//            }
//
//            /* Set the motor power to the variables we've mixed and normalized */
//            leftDrive.setPower(left);
//            rightDrive.setPower(right);
//
//            /* Here we handle the three buttons that have direct control of the intake speed.
//            These control the continuous rotation servo that pulls elements into the robot,
//            If the user presses A, it sets the intake power to the final variable that
//            holds the speed we want to collect at.
//            If the user presses X, it sets the servo to Off.
//            And if the user presses B it reveres the servo to spit out the element.*/
//
//            /* TECH TIP: If Else statements:
//            We're using an else if statement on "gamepad2.x" and "gamepad2.b" just in case
//            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
//            at the same time. "a" will win over and the intake will turn on. If we just had
//            three if statements, then it will set the intake servo's power to multiple speeds in
//            one cycle. Which can cause strange behavior. */
//
//            if (gamepad2.a) {
//                intake.setPower(INTAKE_COLLECT);
//            } else if (gamepad2.x) {
//                intake.setPower(INTAKE_OFF);
//            } else if (gamepad2.b) {
//                intake.setPower(INTAKE_DEPOSIT);
//            }
//
//            /* If left_bumper is pressed, we move the wrist to the left side.
//            If y is pressed, we center the wrist.
//            */
//
//            if (gamepad2.left_bumper) {
//                wrist.setPosition(WRIST_FOLDED_LEFT);
//            } else if (gamepad2.y) {
//                wrist.setPosition(WRIST_FOLDED_CENTER);
//            }
//
//            if (gamepad2.left_stick_y >= -0.1 && gamepad2.left_stick_y <= 0.1) {
//                /* We are using wasUsingPower to keep the arm at it's last position before it went into the idle range.
//                This happens by setting wasUsingPower to TRUE when we are using setPower, then when the joystick goes into the idle range,
//                we check if wasUsingPower is true, which is true the first time it transfers from being outside the range to inside the range.
//                If it is true, we set armLastPosition to the current arm position, then we set wasUsingPower to false, so that we don't call it again
//                while joystick is still inside idle range. When the joystick goes outside idle range, then we set wasUsingPower to true again.
//                Allowing our idle range statement to be able to function correctly the next time.
//                */
//
//                if (wasUsingPower) {
//                    armLastPosition = armMotor.getCurrentPosition();
//                    wasUsingPower = false;
//                }
//
//                armMotor.setTargetPosition(armLastPosition);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else {
//                wasUsingPower = true;
//                armMotor.setPower(gamepad2.left_stick_y);
//                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//
//            /* Check to see if our arm is over the current limit, and report via telemetry. */
//            if (((DcMotorEx) armMotor).isOverCurrent()) {
//                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
//            }
//
//            /* Send telemetry to the driver of the arm's current position and target position */
//            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
//            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
//            telemetry.addData("left_stick_y", gamepad2.left_stick_y);
//            telemetry.addData("armLastPosition", armLastPosition);
//            telemetry.addData("wasUsingPower", wasUsingPower);
//            telemetry.update();

        telemetry.addData("Range: ", gamepad2.left_stick_y >= -0.1 && gamepad2.left_stick_y <= 0.1);
        telemetry.addData("armLastPosition: ", armLastPosition);
        telemetry.addData("currentPosition: ", arm.get_lift_position());
        telemetry.addData("targetPosition: ", arm.get_arm_target_position());
        telemetry.addData("CurrentPositionReal: ", arm.arm_lift.getCurrentPosition());
    }
}