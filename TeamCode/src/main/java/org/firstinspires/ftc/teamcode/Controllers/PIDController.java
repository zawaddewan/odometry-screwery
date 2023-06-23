package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double kP, kD, kI, filterGain,
            lastFilterEstimate, tolerance, target,
            lastError, integral, maxIntegral;

    ElapsedTime timer;

    public PIDController(double kP, double kD, double kI, double filterGain, double tolerance) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.filterGain = filterGain;
        this.tolerance = tolerance;
        timer = new ElapsedTime();
        maxIntegral = Integer.MAX_VALUE;
        lastError = 0;
        lastFilterEstimate = 0;
    }

    public PIDController(double kP, double kD, double kI) {
        this(kP, kD, kI, 0, 0);
    }

    public double update(double measurement) {
        double error = target - measurement;
        return calcFeedback(error);
    }

    public double calcFeedback(double error) {
        double dt = timer.milliseconds();
        double output = error * kP + integrate(error, dt) + differentiate(error, dt);
        lastError = error;
        timer.reset();
        if(Math.abs(error) > tolerance) {
            return output;
        }else {
            return 0;
        }
    }
    //calculate integral plant
    public double integrate(double error, double dt) {
        integral += error * dt;
        //if integral exceeds the maximum allowed integral sum, clip it to the maxIntegral
        if(Math.abs(integral) > Math.abs(maxIntegral)) {
            integral = maxIntegral * Math.signum(integral);
        }
        //if we've crossed beyond the target, reset the integral
        if(Math.signum(error) != Math.signum(lastError)) {
            integral = 0;
        }
        return integral;
    }
    //calculate derivative plant using filtered data
    public double differentiate(double error, double dt) {
        //low pass filter
        double filterEstimate = (filterGain * lastFilterEstimate) + (1 - filterGain) * (error - lastError);
        lastFilterEstimate = filterEstimate;
        return filterEstimate / dt;
    }

    //sets the target for the controller
    public void setTarget(double target) {
        if(target != this.target) {
            integral = 0; //if the target has changed, reset the integral component
        }
        this.target = target;
    }

    //set the allowed tolerance for the controller
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    //set the maximum integral sum allowed for the controller
    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }

    //set the gain for the low pass filter used in the derivative plant
    public void setFilterGain(double filterGain) {
        this.filterGain = filterGain;
    }

    //set the PID coefficients for the controller
    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

}