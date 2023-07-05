package org.firstinspires.ftc.teamcode.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Controllers.AngleController;
import org.firstinspires.ftc.teamcode.Controllers.PIDController;
import static org.firstinspires.ftc.teamcode.Utils.Utils.*;

@Config
public class PoseController {
    SimpleMatrix target;
    SimpleMatrix error;

    public static PIDController xControl;
    public static PIDController yControl;
    public static AngleController thetaControl;

    //instantiate pose controller using a 3x3 matrix of PID coefficients and a column matrix of tolerances
    public PoseController(SimpleMatrix coefficients, SimpleMatrix tolerance) {
        xControl = new PIDController(coefficients.get(0, 0), coefficients.get(0, 1), coefficients.get(0, 2), 0, tolerance.get(0));
        yControl = new PIDController(coefficients.get(1, 0), coefficients.get(1, 1), coefficients.get(1, 2), 0, tolerance.get(1));
        thetaControl = new AngleController(coefficients.get(2, 0), coefficients.get(2, 1), coefficients.get(2, 2), 0, tolerance.get(2));
    }

    //instantiate a pose controller using a 3x3 2-D array of PID coefficients and a length 3 array of tolerances
    public PoseController(double[][] coefficients, double[] tolerance) {
        xControl = new PIDController(coefficients[0][0], coefficients[0][1], coefficients[0][2], 0, tolerance[0]);
        yControl = new PIDController(coefficients[1][0], coefficients[1][1], coefficients[1][2], 0, tolerance[1]);
        thetaControl = new AngleController(coefficients[2][0], coefficients[2][1], coefficients[2][2], 0, tolerance[2]);
    }

    //instantiate a pose controller using pre-instantiated PID controllers
    public PoseController(PIDController xControl, PIDController yControl, AngleController thetaControl) {
        this.xControl = xControl;
        this.yControl = yControl;
        this.thetaControl = thetaControl;
    }

    //set the target for the pose controller (global coordinates)
    public void setTarget(SimpleMatrix target) {
        this.target = target;
    }

    //calculates PID feedback and stores it in matrix form
    public SimpleMatrix calcFeedback(double pose) {
        error = target.minus(pose);
        return new SimpleMatrix(new double[]{
                xControl.calcFeedback(error.get(0)),
                yControl.calcFeedback(error.get(1)),
                thetaControl.calcFeedbackWrap(error.get(2))
        });
    }

    //calculates PID feedback, rotates it to robot coordinates, and then uses inverse kinematics to generate wheel velocities
    public SimpleMatrix update(double pose) {
        SimpleMatrix feedback = calcFeedback(pose);
        SimpleMatrix rotator = SimpleMatrix.wrap(genRotate(feedback.get(2), true));
        return inverseKinematicTransform(rotator.mult(feedback));
    }


}
