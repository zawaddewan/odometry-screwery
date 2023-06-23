package org.firstinspires.ftc.teamcode.Controllers;
import static org.firstinspires.ftc.teamcode.Utils.Utils.*;
public class AngleController extends PIDController{
    public AngleController(double kP, double kD, double kI, double filterGain, double tolerance) {
        super(kP, kD, kI, filterGain, tolerance);
    }

    public AngleController(double kP, double kD, double kI) {
        super(kP, kD, kI);
    }

    @Override
    public double update(double measurement) {
        double error = angleWrapRad(target - measurement);
        return calcFeedback(error);
    }

    public double updateDegrees(double degrees) {
        return update(Math.toRadians(degrees));
    }
}
