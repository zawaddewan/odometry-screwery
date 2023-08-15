package org.firstinspires.ftc.teamcode.Drivetrain.Localizer.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Drivetrain.DriveTrain;

@Config
@Autonomous(name = "Test Localizer", group = "Autonomous")
public class LocalizerTuner extends LinearOpMode {
    // Create drivetrain object
//    DriveTrain drivetrain = new DriveTrain(hardwareMap, true);
    FtcDashboard dashboard;

    /** Public static makes it appear in dashboard**/
    /** Goes from -1 to 1**/
    public static double power = 0;

    /**Existing modes: 0, 1, 2**/
    public static int mode = 0; // 0: +x, 1: +y, 2: +theta

    public static boolean updatePose = false;


    @Override
    public void runOpMode() {
        PhotonCore.enable();

        DriveTrain drivetrain = new DriveTrain(hardwareMap, true);
        // Set dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        /** Measures how fast each timestep */
        ElapsedTime looptime = new ElapsedTime();

        /**Press Start on dashboard*/
        waitForStart();

        /** Reset Timer */
        looptime.reset();

        while (opModeIsActive()) {
            drivetrain.updatePose();

            drivetrain.controlDT(gamepad1);

            /**Telemetry**/
//            double heading = drivetrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            telemetry.addLine("Looptime [ms]: " + looptime.milliseconds());
            telemetry.addLine("Horizontal Encoder: " + (drivetrain.localizer.midPos));
            telemetry.addLine("X: " + (drivetrain.pose.get(0, 0)));
            telemetry.addLine("Y: " + (drivetrain.pose.get(1, 0)));
            telemetry.addLine("Theta: " + Math.toDegrees(drivetrain.pose.get(2, 0)));

            telemetry.addData("IMU Theta", Math.toDegrees(drivetrain.localizer.thetaIMU));
            telemetry.addData("Deadwheel Theta", Math.toDegrees(drivetrain.localizer.thetaDW));
            telemetry.addData("Filtered Theta", Math.toDegrees(drivetrain.localizer.thetaFiltered));

            telemetry.addData("Kalman gain", drivetrain.localizer.kalmanFilter.K);

            telemetry.update();


            looptime.reset();
        }
    }
}
