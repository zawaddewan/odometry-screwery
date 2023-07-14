package org.firstinspires.ftc.teamcode.Motor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.equation.IntegerSequence;

@Config
public class Motor {
    public DcMotorEx motor;
    double epsilon = 0;
    double power = 0;

    public enum ZeroPowerMode {
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT);

        public DcMotor.ZeroPowerBehavior mode;

        ZeroPowerMode(DcMotor.ZeroPowerBehavior mode) {
            this.mode = mode;
        }
    }

    public Motor(HardwareMap hwMap, String name) {
        motor = hwMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public Motor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setEpsilon(double input) {
        epsilon = input;
    }

    public void setPower(double input) {
        if (Math.abs(power - input) > epsilon) { // if abs diff in power is greater than epsilon then set motor power to input
            motor.setPower(input);
            power = input;
        }
    }

    public double getPos() { // gets the encoder positions
        return motor.getCurrentPosition();
    }

    public void setReversed(boolean reversed) {
        if(reversed) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public boolean isReversed() {
        return motor.getDirection().equals(DcMotorSimple.Direction.REVERSE);
    }

    public void setZeroPowerMode(ZeroPowerMode mode) {
        if(mode.equals(ZeroPowerMode.BRAKE)) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}