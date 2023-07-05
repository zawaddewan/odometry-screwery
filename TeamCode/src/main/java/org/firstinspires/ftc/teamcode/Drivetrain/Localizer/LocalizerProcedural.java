package org.firstinspires.ftc.teamcode.Drivetrain.Localizer;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Motor.Motor;
import static org.ejml.dense.fixed.CommonOps_DDF3.*;
import static org.firstinspires.ftc.teamcode.Utils.Utils.*;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LocalizerProcedural {

    Motor leftEncoder;
    Motor midEncoder;
    Motor rightEncoder;

    double leftPos = 0;
    double midPos = 0;
    double rightPos = 0;

    public static double trackWidth = 1; // mm
    public static double forwardOffset = 1; // mm
    public static double mmPerTick = 694; // (mm / rev) / (ticks / rev) = mm / ticks
    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    //magic matrix that Converts processed wheel positions to relative robot pose change
    DMatrix3x3 C;

    public LocalizerProcedural(Motor leftEncoder, Motor rightEncoder, Motor midEncoder) {
        //instantiate motors
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.midEncoder = midEncoder;

        /*
        create conversion matrix, this is done in the constructor so that if an opMode is reinitialized,
        any changes to the trackWidth or forwardOffset will be reflected in this matrix without the need to push code
        */
        C = new DMatrix3x3(
                0.5, 0.5, 0,
                -1 * forwardOffset / trackWidth, forwardOffset / trackWidth, 1,
                1 / trackWidth, -1 / trackWidth, 0
        );
    }

    public DMatrix3 calcDelOdo() {
        //grab encoder positions
        double newLeftPos = leftEncoder.getPos();
        double newRightPos = rightEncoder.getPos();
        double newMidPos = midEncoder.getPos();

        //calculate difference in current and new encoder positions, convert from ticks to mm, store in column vector
        DMatrix3 delOdo = new DMatrix3(
                (newLeftPos - leftPos) * mmPerTick * X_MULTIPLIER,
                (newRightPos - rightPos) * mmPerTick * X_MULTIPLIER,
                (newMidPos - midPos) * mmPerTick * Y_MULTIPLIER
        );

        //save new encoder positions
        leftPos = newLeftPos;
        rightPos = newRightPos;
        midPos = newMidPos;

        return delOdo;
    }

    public DMatrix3x3 calcPoseExp(double angle) {
        //calculate the pose exponential matrix
        try {
            return new DMatrix3x3(
                    Math.sin(angle) / angle, (Math.cos(angle) - 1) / angle, 0,
                    (1 - Math.cos(angle)) / angle, Math.sin(angle) / angle, 0,
                    0, 0, 1
            );
        } catch (ArithmeticException e) {
            //if angle is 0, return identity matrix
            return new DMatrix3x3(
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1
            );
        }
    }

    public DMatrix3 calcDelPose(double theta) {
        DMatrix3 delG = new DMatrix3();
        DMatrix3 O = calcDelOdo();
        //convert odometry
        mult(C, O, delG);
        //multiply by pose exp
        mult(calcPoseExp(delG.unsafe_get(2, 0)), delG, O);
        //rotate
        mult(genRotate(theta, false), O, delG);
        return delG;
    }

}
