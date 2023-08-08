package org.firstinspires.ftc.teamcode.Drivetrain.Localizer;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Motor.Motor;
import static org.ejml.dense.fixed.CommonOps_DDF3.*;
import static org.firstinspires.ftc.teamcode.Utils.Utils.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LocalizerProcedural {

    Motor leftEncoder;
    Motor midEncoder;
    Motor rightEncoder;

    public double leftPos = 0;
    public double midPos = 0;
    public double rightPos = 0;

    public static double forwardOffset = 0.15575; // m
    public static double trackWidth = 0.409575; // m
    public static double ticksPerRev = 8192;
    public static double odoWheelDiameter = 0.035; //m
    public static double mPerTick = odoWheelDiameter * Math.PI / ticksPerRev; // (m / rev) / (ticks / rev) = m / ticks
    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;


    //magic matrix that Converts processed wheel positions to relative robot pose change
    DMatrix3x3 C;

    public LocalizerProcedural(HardwareMap hardwareMap) {
        leftEncoder = new Motor(hardwareMap, "leftFront");
        rightEncoder = new Motor(hardwareMap, "rightFront");
        midEncoder = new Motor(hardwareMap, "leftBack");

        //reverse any encoders if necessary
        leftEncoder.reverseEncoder(true);

        C = new DMatrix3x3(
                0.5, 0.5, 0.0,
                -1.0 * forwardOffset / trackWidth, forwardOffset / trackWidth, 1.0,
                1.0 / trackWidth, -1.0 / trackWidth, 0.0
        );
    }

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
                0.5, 0.5, 0.0,
                -1.0 * forwardOffset / trackWidth, forwardOffset / trackWidth, 1.0,
                1.0 / trackWidth, -1.0 / trackWidth, 0.0
        );
    }

    public DMatrix3 calcDelOdo() {
        //grab encoder positions
        double newLeftPos = leftEncoder.getPos();
        double newRightPos = rightEncoder.getPos();
        double newMidPos = midEncoder.getPos();

        //calculate difference in current and new encoder positions, convert from ticks to mm, store in column vector
        DMatrix3 delOdo = new DMatrix3(
                (newLeftPos - leftPos) * mPerTick * X_MULTIPLIER,
                (newRightPos - rightPos) * mPerTick * X_MULTIPLIER,
                (newMidPos - midPos) * mPerTick * Y_MULTIPLIER
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
        DMatrix3 delG = new DMatrix3(0, 0, 0);
        DMatrix3 O = calcDelOdo();
        //convert odometry
        mult(C, O, delG);
        //multiply by pose exp
        mult(calcPoseExp(delG.unsafe_get(2, 0)), delG, O);
        //rotate
        mult(genRotate(theta, false), O, delG);
        return delG;
    }

    public SimpleMatrix update(SimpleMatrix pose) {
        DMatrix3 delta = calcDelPose(pose.get(2, 0));
        SimpleMatrix simpleDelta = new SimpleMatrix(new double[]{
                delta.get(0, 0),
                delta.get(1, 0),
                delta.get(2, 0),
        });
        return pose.plus(simpleDelta);
    }

}
