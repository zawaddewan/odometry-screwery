package org.firstinspires.ftc.teamcode.Localizer;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Motor.Motor;

public class Localizer {
    Motor leftEncoder;
    Motor midEncoder;
    Motor rightEncoder;

    double leftPos = 0;
    double midPos = 0;
    double rightPos = 0;

    public static double trackWidth = 1; // mm
    public static double forwardOffset = 1; // mm
    public static double mmPerTick = 694; // (mm / rev) / (ticks / rev) = mm / ticks

    //magic matrix that Converts processed wheel positions to relative robot pose change
    SimpleMatrix C;

    public Localizer(Motor leftEncoder, Motor rightEncoder, Motor midEncoder) {
        //instantiate motors
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.midEncoder = midEncoder;

        /*
        create conversion matrix, this is done in the constructor so that if an opMode is reinitialized,
        any changes to the trackWidth or forwardOffset will be reflected in this matrix without the need to push code
        */
        C = new SimpleMatrix(new double[][]{
                new double[]{0.5, 0.5, 0},
                new double[]{-1 * forwardOffset / trackWidth, forwardOffset / trackWidth, 1},
                new double[]{1 / trackWidth, -1 / trackWidth, 0},
        });
    }

    public SimpleMatrix calcDelOdo() {
        //grab encoder positions
        double newLeftPos = leftEncoder.getPos();
        double newRightPos = rightEncoder.getPos();
        double newMidPos = midEncoder.getPos();

        //calculate difference in current and new encoder positions, convert from ticks to mm, store in column vector
        SimpleMatrix delOdo = new SimpleMatrix(new double[]{
                (newLeftPos - leftPos) * mmPerTick,
                (newRightPos - rightPos) * mmPerTick,
                (newMidPos - midPos) * mmPerTick
        });

        //save new encoder positions
        leftPos = newLeftPos;
        rightPos = newRightPos;
        midPos = newMidPos;

        return delOdo;
    }

    public SimpleMatrix calcPoseExp(double angle) {
        //pose exponential is identity matrix by default, in case angle is zero
        SimpleMatrix PoseExp = SimpleMatrix.identity(3);

        //if angle is not 0, calculate the pose exponential matrix
        if (angle != 0) {
            PoseExp = new SimpleMatrix(new double[][]{
                    new double[]{Math.sin(angle) / angle, (Math.cos(angle) - 1) / angle, 0},
                    new double[]{(1 - Math.cos(angle)) / angle, Math.sin(angle) / angle, 0},
                    new double[]{0, 0, 1},
            });
        }
        return PoseExp;
    }

    public SimpleMatrix calcDelRobot() {
        //multiplying the conversion matrix with the odometry wheel matrix returns the matrix of the relative robot pose change
        SimpleMatrix delRobot = C.mult(calcDelOdo());

        /*
        calculate the pose exponential and then multiply it with the robot pose change matrix
        entry (3,1) in the robot pose change matrix is the change in heading, which is used to calculate the pose exponential
        */
        return calcPoseExp(delRobot.get(3, 1)).mult(delRobot);
    }



    public SimpleMatrix calcDelGlobal(double theta) { //theta is current robot heading
        //calculate rotation matrix given current robot heading
        SimpleMatrix rotation = new SimpleMatrix(new double[][] {
                new double[] {Math.cos(theta), -1 * Math.sin(theta), 0},
                new double[] {Math.sin(theta), Math.cos(theta), 0},
                new double[] {0, 0, 1},
        });

        //calculate relative change in robot pose and rotate by the current robot heading
        return rotation.mult(calcDelRobot());
    }
}