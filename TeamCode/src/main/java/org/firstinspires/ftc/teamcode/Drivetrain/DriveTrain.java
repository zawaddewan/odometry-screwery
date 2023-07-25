package org.firstinspires.ftc.teamcode.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Drivetrain.Controllers.PoseController;
import org.firstinspires.ftc.teamcode.Drivetrain.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.Motor.Motor;

@Config
public class DriveTrain {

    //POSE CONTROLLER RELATED THINGS
    PoseController controller;

    public Localizer localizer;

    public static double xKP = 1;
    public static double xKI = 0;
    public static double xKD = 0;
    public static double yKP = 1;
    public static double yKI = 0;
    public static double yKD = 0;
    public static double thetaKP = 1;
    public static double thetaKI = 0;
    public static double thetaKD = 0;

    public static double xTolerance = 0;
    public static double yTolerance = 0;
    public static double thetaTolerance = 0;

    public static double strafeMultiplier = 1;
    public static double kV = 1;

    public SimpleMatrix pose;

    //MOTOR RELATED THINGS
    Motor[] motors;

    Motor leftFront, leftBack, rightFront, rightBack;

    public static double motorEpsilon = 0; //motor power setting threshold value, see Motor.java


    public DriveTrain(HardwareMap hardwareMap, boolean bulkReads) {
        //toggles bulk reads if desired
        if(bulkReads) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        //instantiate motors
        leftFront = new Motor(hardwareMap, "leftFront");
        leftBack = new Motor(hardwareMap, "leftBack");
        rightFront = new Motor(hardwareMap, "rightFront");
        rightBack = new Motor(hardwareMap, "rightBack");

        //keeping the motors in an array is helpful
        motors = new Motor[]{leftFront, leftBack, rightBack, rightFront};

        //reverse motors
        rightBack.setReversed(true);
        rightFront.setReversed(true);

        //configure motors
        for(Motor motor : motors) {
            motor.setEpsilon(motorEpsilon);
            motor.setZeroPowerMode(Motor.ZeroPowerMode.BRAKE);
        }

        //instantiate localizer
        localizer = new Localizer(hardwareMap);

        //column vector of 0s aka (0, 0, 0) for (x, y, theta)
        pose = new SimpleMatrix(new double[]{0, 0, 0});

        //instantiate pose controller with PID coefficients and tolerances
        double[][] coefficients = new double[][] {
                new double[]{xKP, xKI, xKD},
                new double[]{yKP, yKI, yKD},
                new double[]{thetaKP, thetaKI, thetaKD}};

        double[] tolerances = new double[]{xTolerance, yTolerance, thetaTolerance};

        controller = new PoseController(coefficients, tolerances);
    }

    //sets powers given a 4 element column vector of motor powers and a feedforward component
    public void setPower(SimpleMatrix powers, double kF) {
        /*
        toArray2 turns the matrix into a 2D array
        powers.toArray2()[i][0] accesses the ith element in the column vector
         */
        for(int i = 0; i < 4; i++) {
            motors[i].setPower(kF * powers.toArray2()[i][0]);
        }
    }

    //sets powers, assuming no feedforward control
    public void setPower(SimpleMatrix powers) {
        setPower(powers, 1);
    }


    //calculate motor powers that correspond to current gamepad inputs and run the motors
    public void controlDT(Gamepad gamepad) {
        double x = -gamepad.left_stick_x * strafeMultiplier;
        double y = gamepad.left_stick_y;
        double theta = gamepad.right_stick_x;

        double scaleFactor = 1 / Math.max(Math.abs(x) + Math.abs(y) + Math.abs(theta), 1);

        SimpleMatrix powers = new SimpleMatrix(new double[]{
                x + y + theta,
                -x + y + theta,
                x + y - theta,
                -x + y - theta
        }).scale(scaleFactor);

        setPower(powers);
    }

    //sets the robot's current pose (global coordinates) given a 3 element column vector representing (x, y, theta)
    public void setPose(SimpleMatrix pose) {
        this.pose = pose;
    }

    //sets the robot's current pose (global coordinates) given an x, y, and theta
    public void setPose(double x, double y, double theta) {
        pose = new SimpleMatrix(new double[]{
                x,
                y,
                theta
        });
    }

    //returns the current robot pose (global coordinates)
    public SimpleMatrix getPose() {
        return pose;
    }

    //runs the localizer
    public void updatePose() {
        pose = localizer.update(pose);
    }

    //sets target for pose controller given a 3 element column vector representing (x, y, theta)
    public void setTarget(SimpleMatrix target) {
        controller.setTarget(target);
    }

    //sets the target for the pose controller given coordinates and angle
    public void setTarget(double x, double y, double theta) {
        controller.setTarget(new SimpleMatrix(new double[]{
                x,
                y,
                theta,
        }));
    }

    //updates pose and runs the pose controller
    public void run() {
        updatePose();
        setPower(controller.update(pose), kV);
    }

}
