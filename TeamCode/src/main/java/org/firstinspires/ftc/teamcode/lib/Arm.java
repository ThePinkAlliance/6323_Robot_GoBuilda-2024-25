package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public DcMotor arm_lift;
    public DcMotorEx arm_extend_motor;
    public Servo arm_rotate;
    public CRServo arm_collector;
    public Arm(HardwareMap map) {
        this.arm_extend_motor = map.get(DcMotorEx.class, "arm_extend");
        this.arm_lift = map.get(DcMotor.class, "arm_lift");
        this.arm_rotate = map.get(Servo.class, "arm_rotate");
        this.arm_collector = map.get(CRServo.class, "arm_collect");

        this.arm_rotate.setDirection(Servo.Direction.FORWARD);

        this.arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.arm_extend_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm_extend_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.arm_lift.setTargetPosition(0);
        this.arm_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm_lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int get_lift_position() {
        return this.arm_lift.getCurrentPosition();
    }

    public double get_extend_position() {
        return this.arm_extend_motor.getCurrentPosition();
    }

    public double get_rotate_position() {
        return this.arm_rotate.getPosition();
    }

    public double get_arm_target_position() {
        return this.arm_lift.getTargetPosition();
    }

    public void set_extend_position(int position) {
        this.arm_extend_motor.setTargetPosition(position);
    }

    public void set_rotate_position(double position) {
        this.arm_rotate.setPosition(position);
    }

    public void set_lift_position(int position) {
        this.arm_lift.setTargetPosition(position);
    }

    public void set_lift_power(double power) {
        this.arm_lift.setPower(power);
    }

    public void set_extend_power(double power) {
        this.arm_extend_motor.setPower(power);
    }

    public void set_collect_power(double power) {
        this.arm_collector.setPower(power);
    }

    public void set_extend_mode(DcMotor.RunMode mode) {
        this.arm_extend_motor.setMode(mode);
    }

    public void set_arm_mode(DcMotor.RunMode mode) {
        this.arm_lift.setMode(mode);
    }
}
