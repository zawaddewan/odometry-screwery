package org.firstinspires.ftc.teamcode.Drivetrain.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Drivetrain.DriveTrain;

/**
 THIS IS AN AUTONOMOUS OPMODE WE WILL USE TO TEST
 YOUR DRIVETRAIN'S MOTOR DIRECTIONS.
 MAKE SURE YOU ADD THE @CONFIG AT THE TOP OF ALL
 YOUR TUNING/TESTING OPMODES. FTC DASHBOARD IS
 BETTER THAN PRINTING ONTO THE PHONE VIA FTC SDK
 TELEMETRY. DASHBOARD TELEMETRY IS BETTER!
 */
@Config
@Autonomous(name = "Test Motors", group = "Autonomous")
public class TestMotors extends LinearOpMode {
    // Create drivetrain object
    /**
     SINCE THIS IS A TEST WITHIN OUR DRIVETRAIN FOLDER, WE INSTANTIATE
     THE DRIVETRAIN DIRECTLY IN THE OP MODE.
     */
    DriveTrain drivetrain;

    // Use FTCDashboard
    /**
     HERE WE MUST ALSO ESTABLISH THE FTC DASHBOARD OBJECT.
     */
    FtcDashboard dashboard;

    /**
     THIS IS VERY IMPORTANT. TO GET OUR TUNABLE/TESTABLE
     VARIABLES TO SHOW UP ON OUR FTC DASHBOARD, WE MUST
     USE PUBLIC & STATIC!!
     WHEN WE CONNECT TO THE ROBOT, WE CAN CHANGE THIS VARIABLE
     FROM ANYWHERE BETWEEN -1 AND 1 TO TEST DIFFERENT DIRECTIONS.
     */
    public static double power = 0;


    /**
     HERE WE CAN ESTABLISH ANOTHER TESTABLE VARIABLE, BUT IT
     IS AN INT SO MAKE SURE YOU ONLY CHANGE IT TO INTS FROM 0 TO 2!
     */
    public static int mode = 0; // 0: +x, 1: +y, 2: +theta


    @Override
    public void runOpMode() {
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


        /**
         WE CAN USE THIS TO GET AN IDEA OF HOW FAST EACH
         TIME-STEP/LOOP ITERATION RUNS.
         */
        ElapsedTime looptime = new ElapsedTime();



        /**
         HERE ARE SOME MATRICES THAT CORRESPOND TO SPECIFIC TYPES
         OF MOVEMENTS. YOU CAN REFER TO OUR KINEMATIC MODELS, BUT FOR
         REFERENCE, THE ORDER IS AS FOLLOWS:
         0: LEFT-FRONT
         1: LEFT-BACK
         2: RIGHT-BACK
         3: RIGHT-FRONT
         YOU SHOULD ALWAYS ESTABLISH YOUR POWER VECTORS IN THIS ORDER AS
         THE ROBOT'S setPower FUNCTION SETS POWER TO MOTORS IN THAT
         CONFIGURATION
         THE FIRST MATRIX SHOULD CAUSE THE ROBOT TO GO STRAIGHT (FORWARDS
         IF YOUR power VARIABLE IS GREATER THAN 0 AND BACKWARDS IF LESS THAN 0)
         THE SECOND MATRIX SHOULD CAUSE THE ROBOT TO STRAFE (TO THE LEFT
         IF YOUR power VARIABLE IS GREATER THAN 0 AND BACKWARDS IF LESS THAN 0)
         THE THIRD MATRIX SHOULD CAUSE THE ROBOT TO TURN (COUNTERCLOCKWISE
         IF YOUR power VARIABLE IS GREATER THAN 0 AND CLOCKWISE IF LESS THAN 0)
         */
        SimpleMatrix uStraight = new SimpleMatrix(new double[][]{
                new double[]{power},
                new double[]{power},
                new double[]{power},
                new double[]{power}

        });
        SimpleMatrix uStrafe = new SimpleMatrix(new double[][]{
                new double[]{power},
                new double[]{-power},
                new double[]{power},
                new double[]{-power}

        });
        SimpleMatrix uTurn = new SimpleMatrix(new double[][]{
                new double[]{-power},
                new double[]{-power},
                new double[]{power},
                new double[]{power}

        });


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
            /**
             BASED ON THE MODE, WE PICK THE MATRIX TO GIVE TO
             THE DRIVETRAIN'S setPower FUNCTION!
             */
            if (mode == 0) {
                drivetrain.setPower(uStraight);
            } else if (mode == 1) {

                drivetrain.setPower(uStrafe);
            } else if (mode == 2) {
                drivetrain.setPower(uTurn);
            }

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
            telemetry.addData("Looptime [ms]", looptime.milliseconds());
            telemetry.update();

            looptime.reset();
        }
    }
}