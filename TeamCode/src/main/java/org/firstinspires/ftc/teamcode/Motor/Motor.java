package org.firstinspires.ftc.teamcode.Motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    DcMotorEx motor;
    double epsilon = 0;
    double power = 0;

    public Motor(HardwareMap hwMap, String name) {
        motor = hwMap.get(DcMotorEx.class, name);
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
}