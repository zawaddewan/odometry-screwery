package org.firstinspires.ftc.teamcode.Utils;

import org.ejml.data.DMatrix3x3;
import org.ejml.simple.SimpleMatrix;

public class Utils {

    //drivetrain stats
    public static double wheelBase = 1;
    public static double trackWidth = 1;
    public static double wheelRadius = 1;

    //m is the 'mecanum constant', the average of the wheelbase and track width
    public static double m = (wheelBase + trackWidth) / 2;

    //Forward Kinematic Matrix
    static SimpleMatrix FK = new SimpleMatrix(new double[][]{
            new double[]{1, 1, 1, 1},
            new double[]{-1, 1, -1, 1},
            new double[]{-1 / m, -1 / m, 1 / m, 1 / m},

    });

    //Inverse Kinematic Matrix
    static SimpleMatrix IK = new SimpleMatrix(new double[][]{
            new double[]{1, -1, -m},
            new double[]{1, 1, -m},
            new double[]{1, -1, m},
            new double[]{1, 1, m}
    });

    //computes robot velocities given wheel velocities
    public static SimpleMatrix forwardKinematicTransform(SimpleMatrix wheelVelocities) {
        return FK.scale(wheelRadius / 4).mult(wheelVelocities);
    }

    //computes wheel velocities given robot velocities
    public static SimpleMatrix inverseKinematicTransform(SimpleMatrix robotVelocities) {
        return IK.scale(1 / wheelRadius).mult(robotVelocities);
    }

    //returns sinc(x) = sin(x) / x. equals 1 at 0
    public static double sinc(double x) {
        try {
            return Math.sin(x) / x;
        } catch (ArithmeticException e) {
            return 1;
        }
    }

    //generates rotation matrix for either clockwise (robot to global) or counterclockwise (global to robot)
    public static DMatrix3x3 genRotate(double theta, boolean clockwise) {
        if(clockwise) {theta *= -1;}
        return new DMatrix3x3(
                Math.cos(theta), -1 * Math.sin(theta), 0,
                Math.sin(theta), Math.cos(theta), 0,
                0, 0, 1
        );
    }

    //wraps an angle from -180 to 180 degrees
    public static double angleWrapDeg(double degrees) {
        double wrap = (degrees + 180) % 360;
        if (wrap < 0) {
            wrap += 360;
        }
        return wrap - 180;
    }

    //same as above but for radians
    public static double angleWrapRad(double radians) {
        return Math.toRadians(angleWrapDeg(Math.toDegrees(radians)));
    }


}
