package org.firstinspires.ftc.teamcode.Drivetrain.Localizer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utils.Utils.*;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Motor.Motor;

@Config
public class Localizer {
    Motor leftEncoder;
    Motor midEncoder;
    Motor rightEncoder;

    public double leftPos = 0;
    public double midPos = 0;
    public double rightPos = 0;

    public static double forwardOffset = 0.15575; // m
    public static double trackWidth = 0.4191; // m
    public static double ticksPerRev = 8192;
    public static double odoWheelDiameter = 0.035; //m
    public static double mPerTick = odoWheelDiameter * Math.PI / ticksPerRev; // (m / rev) / (ticks / rev) = m / ticks
    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    //magic matrix that Converts processed wheel positions to relative robot pose change
    SimpleMatrix C;

    public static boolean poseExponential = true;

    public double timeOfLastCalc = 0; //ms
    ElapsedTime timer;

    public Localizer(HardwareMap hardwareMap) {
        //instantiate "motors"
        leftEncoder = new Motor(hardwareMap, "leftFront");
        rightEncoder = new Motor(hardwareMap, "rightFront");
        midEncoder = new Motor(hardwareMap, "leftBack");

        //reverse any encoders if necessary
        leftEncoder.reverseEncoder(true);

        /*
        create conversion matrix, this is done in the constructor so that if an opMode is reinitialized,
        any changes to the trackWidth or forwardOffset will be reflected in this matrix without the need to push code
        */
        C = new SimpleMatrix(new double[][]{
                new double[]{0.5, 0.5, 0},
                new double[]{-1 * forwardOffset / trackWidth, forwardOffset / trackWidth, 1},
                new double[]{1 / trackWidth, -1 / trackWidth, 0},
        });

        timer = new ElapsedTime();
    }

    public Localizer(Motor leftEncoder, Motor rightEncoder, Motor midEncoder) {
        //instantiate "motors"
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.midEncoder = midEncoder;

        /*
        create conversion matrix, this is done in the constructor so that if an opMode is reinitialized,
        any changes to the trackWidth or forwardOffset will be reflected in this matrix without the need to push code
        */
        C = new SimpleMatrix(new double[][]{
                new double[]{1, 1, 0},
                new double[]{-1 * forwardOffset / trackWidth, forwardOffset / trackWidth, 1},
                new double[]{1 / trackWidth, -1 / trackWidth, 0},
        });

        timer = new ElapsedTime();
    }

    public SimpleMatrix update(SimpleMatrix pose) {
        double currentTime = timer.milliseconds();
        SimpleMatrix deltas = calcDelGlobal(pose.get(2, 0));
        timeOfLastCalc = timer.milliseconds() - currentTime;
        return pose.plus(deltas);
    }

    public SimpleMatrix calcDelOdo() {
        //grab encoder positions
        double newLeftPos = leftEncoder.getPos();
        double newRightPos = rightEncoder.getPos();
        double newMidPos = midEncoder.getPos();

        //calculate difference in current and new encoder positions, convert from ticks to mm, store in column vector
        SimpleMatrix delOdo = new SimpleMatrix(new double[]{
                (newLeftPos - leftPos) * mPerTick * X_MULTIPLIER,
                (newRightPos - rightPos) * mPerTick * X_MULTIPLIER,
                (newMidPos - midPos) * mPerTick * Y_MULTIPLIER
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
        if (angle != 0 && poseExponential) {
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

        SimpleMatrix delOdo = calcDelOdo();

//        SimpleMatrix delRobot = C.mult(delOdo);



        SimpleMatrix delRobot = new SimpleMatrix(new double[]{
                (delOdo.get(0, 0) + delOdo.get(1, 0))/2,
                delOdo.get(2, 0) - forwardOffset*((delOdo.get(0, 0) - delOdo.get(1, 0)) / trackWidth),
                (delOdo.get(0, 0) - delOdo.get(1, 0)) / trackWidth
        });

        /*
        calculate the pose exponential and then multiply it with the robot pose change matrix
        entry (3,1) in the robot pose change matrix is the change in heading, which is used to calculate the pose exponential
        */

        return calcPoseExp(delRobot.get(2, 0)).mult(delRobot);
    }



    public SimpleMatrix calcDelGlobal(double theta) { //theta is current robot heading
        //calculate rotation matrix given current robot heading
        SimpleMatrix rotation = genRotateSimple(theta, false);

        //calculate relative change in robot pose and rotate by the current robot heading
        return rotation.mult(calcDelRobot());
    }

    public double getTimeOfLastCalc() {
        return timeOfLastCalc;
    }
}