package org.firstinspires.ftc.teamcode.Utils;

import org.ejml.data.DMatrix3x3;

public class Utils {

    public static double sinc(double x) {
        try {
            return Math.sin(x) / x;
        } catch (ArithmeticException e) {
            return 1;
        }
    }

    public static double cosQuotient(double x) {
        try {
            return (1 - Math.cos(x)) / x;
        } catch (ArithmeticException e) {
            return 0;
        }
    }

    public static DMatrix3x3 genRotate(double theta, boolean clockwise) {
        if(clockwise) {theta *= -1;}
        return new DMatrix3x3(
                Math.cos(theta), -1 * Math.sin(theta), 0,
                Math.sin(theta), Math.cos(theta), 0,
                0, 0, 1
        );
    }
}
