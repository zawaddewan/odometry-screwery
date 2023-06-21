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

    public Localizer(Motor leftEncoder, Motor rightEncoder, Motor midEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.midEncoder = midEncoder;
    }

    public double[] calcDelOdo() {

        //grab encoder positions
        double newLeftPos = leftEncoder.getPos();
        double newMidPos = midEncoder.getPos();
        double newRightPos = rightEncoder.getPos();

        //calculate difference in current and new encoder positions, convert from ticks to mm
        double[] delOdo = new double[]{
                (newLeftPos - leftPos) * mmPerTick,
                (newMidPos - midPos) * mmPerTick,
                (newRightPos - rightPos) * mmPerTick,
        };

        //save new encoder positions
        leftPos = newLeftPos;
        midPos = newMidPos;
        rightPos = newRightPos;

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
        double[] delOdo = calcDelOdo(); //delOdo[0] = change in left encoder, delOdo[1] = change in mid encoder, delOdo[2] = change in right encoder

        //calculate change in angle
        double phi = (delOdo[0] - delOdo[2]) / trackWidth;

        //calculate relative vertical displacement of robot
        double centerDisplacement = (delOdo[0] + delOdo[2]) / 2;

        //calculate relative horizontal displacement of robot
        double horizontalDisplacement = delOdo[1] - (forwardOffset * phi);

        //save values into a matrix
        SimpleMatrix delRobot = new SimpleMatrix(new double[][] {
                new double[] {centerDisplacement},
                new double[] {horizontalDisplacement},
                new double[] {phi},
        });

        //calculate the pose exponential and then multiply the matrices together to get relative change in robot pose
        SimpleMatrix PoseExp = calcPoseExp(phi);
        return PoseExp.mult(delRobot);
    }



    public SimpleMatrix calcDelGlobal(double theta) { //theta is current robot heading
        //calculate rotation matrix given current robot heading
        SimpleMatrix rotation = new SimpleMatrix(new double[][] {
                new double[] {Math.cos(theta), -1 * Math.sin(theta), 0},
                new double[] {Math.sin(theta), Math.cos(theta), 0},
                new double[] {0, 0, 1},
        });

        //calculate relative change in robot pose and rotate by the current robot heading
        SimpleMatrix delRobot = calcDelRobot();
        return rotation.mult(delRobot);
    }
}