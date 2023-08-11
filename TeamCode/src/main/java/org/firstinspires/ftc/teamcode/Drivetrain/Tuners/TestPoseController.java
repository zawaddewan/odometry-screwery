package org.firstinspires.ftc.teamcode.Drivetrain.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Drivetrain.Controllers.PoseController;
import org.firstinspires.ftc.teamcode.Drivetrain.DriveTrain;

@Config
@Autonomous(name = "Test Pose Controller", group = "Autonomous")
public class TestPoseController extends LinearOpMode {

    // Create drivetrain object
    DriveTrain drivetrain;
    PoseController poseController;

    // Use FTCDashboard
    FtcDashboard dashboard;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double thTarget = 1.57;

    public static double kPx = 0.3;
    public static double kIx = 0;
    public static double kDx = 30;

    public static double kPy = 0.3;
    public static double kIy = 0;
    public static double kDy = 30;

    public static double kPth = 1;
    public static double kIth = 0;
    public static double kDth = 30;

    public double[][] coefficients = new double[][] {
            new double[]{kPx, kIx, kDx},
            new double[]{kPy, kIy, kDy},
            new double[]{kPth, kIth, kDth},
    };

    public double[] tolerances = new double[] {0,0,0};


    SimpleMatrix poseTarget = new SimpleMatrix(
            new double[][]{
                    new double[]{xTarget},
                    new double[]{yTarget},
                    new double[]{thTarget}
            }
            );


        @Override
        public void runOpMode() {
            PhotonCore.enable();
            // Set dashboard
            /**
             THESE TWO LINES ARE IMPORTANT TO GET DASHBOARD
             WORKING CORRECTLY!
             */
            dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();

            /**
             WE USE THIS TO SEND OVER THE HARDWARE MAP.
             */
            drivetrain = new DriveTrain(hardwareMap, true);
            poseController= new PoseController(coefficients, tolerances);



            /**
             WE CAN USE THIS TO GET AN IDEA OF HOW FAST EACH
             TIME-STEP/LOOP ITERATION RUNS.
             */
            ElapsedTime looptime = new ElapsedTime();

            telemetry.addData("X", (drivetrain.pose.get(0, 0)));
            telemetry.addData("Y", (drivetrain.pose.get(1, 0)));
            telemetry.addData("Theta", Math.toDegrees(drivetrain.pose.get(2, 0)));

            telemetry.addData("XTarget", (xTarget));
            telemetry.addData("YTarget", (yTarget));
            telemetry.addData("ThetaTarget", Math.toDegrees(thTarget));


            /**
             AS OF NOW, EVERYTHING HAS BEEN HAPPENING IN THE INIT PHASE
             OF THE ROBOT. WE ARE NOW WAITING FOR SOMEONE TO HIT THE START BUTTON
             ON THE PHONE OR ON FTC DASHBOARD!
             */
            waitForStart();

            /**
             WE RESET OUR TIMER IMMIDIATELY!
             */
            looptime.reset();

            while (opModeIsActive()) {

                drivetrain.updatePose();
                /**
                 BASED ON THE MODE, WE PICK THE MATRIX TO GIVE TO
                 THE DRIVETRAIN'S setPower FUNCTION!
                 */
//
                poseController.setTarget(new SimpleMatrix(
                        new double[][] {
                                new double[]{xTarget},
                                new double[]{yTarget},
                                new double[]{thTarget},
                        }
                ));
//                SimpleMatrix poop = poseController.update(drivetrain.getPose());
                drivetrain.setPower(poseController.update(drivetrain.getPose()), 1);

                /**
                 IN THIS CASE THE ONLY THING WE ARE PRINTING/GRAPHING IS
                 THE LOOPTIME IN MILLISECONDS. THIS IS HOW WE CAN ADD
                 DATA TO FTCDASHBOARD (telemetry.addData(...); and then telemetry.update();)
                 YOU CAN USE TELEMETRY TO PRINT/GRAPH YOUR DRIVETRAIN'S POSE BY
                 DOING SOMETHING LIKE:
                 telemetry.addData("x-position [mm]", drivetrain.pose.get(0, 0));
                 telemetry.addData("y-position [mm]", drivetrain.pose.get(1, 0));
                 telemetry.addData("heading [rads]", drivetrain.pose.get(2, 0));
                 HOWEVER WE DON'T HAVE OUR LOCALIZER YET SO IT WILL ALL BE CONSTANT!!!!
                 WE THEN RESET THE LOOPTIMER AGAIN FOR THE NEXT ITERATION!
                 REMEMBER, WE ARE ESSENTIALLY COUNTING THE TIME IT TAKES FOR
                 EACH TIME-STEP!
                 */

                telemetry.addData("Looptime [ms]: ", looptime.milliseconds());
                telemetry.addData("Left Encoder: " , (drivetrain.localizer.leftPos));
                telemetry.addData("Right Encoder: " , (drivetrain.localizer.rightPos));
                telemetry.addData("Horizontal Encoder: " , (drivetrain.localizer.midPos));

                telemetry.addData("X", (drivetrain.pose.get(0, 0)));
                telemetry.addData("Y", (drivetrain.pose.get(1, 0)));
                telemetry.addData("Theta", Math.toDegrees(drivetrain.pose.get(2, 0)));

                telemetry.addData("XTarget", (xTarget));
                telemetry.addData("YTarget", (yTarget));
                telemetry.addData("ThetaTarget", Math.toDegrees(thTarget));

                telemetry.addLine("X: " + (drivetrain.pose.get(0, 0)));
                telemetry.addLine("Y: " + (drivetrain.pose.get(1, 0)));
                telemetry.addLine("Theta: " + Math.toDegrees(drivetrain.pose.get(2, 0)));
                telemetry.addLine("Vx: " + poseController.output.get(0, 0));
                telemetry.addLine("Vy: " + poseController.output.get(1, 0));
                telemetry.addLine("Vth " + poseController.output.get(2, 0));


                telemetry.update();


                looptime.reset();
            }
        }
    }
