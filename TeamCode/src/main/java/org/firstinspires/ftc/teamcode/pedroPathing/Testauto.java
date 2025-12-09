package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Table2D;
import org.firstinspires.ftc.teamcode.flywheelModule;

@Autonomous(name = "test auto", group = "Examples")
public class Testauto extends OpMode {



    public Servo hood;
    public DcMotor flywheel;
    public DcMotor intake;
    public DcMotor transfer;
    float[] distance = {22, 30, 35, 40,44,52,56,69,81,125,126};
    private float[] flywheel_speed = {2650, 2900, 3000, 3100, 3150, 3270, 3300, 3250, 3350, 4000,4000};
    private float[] hood_angle = { (float)0.75, (float)0.75, (float)0.75, (float)0.75, (float)0.75,(float)0.75,(float)0.75,(float)0.75,(float)0.65,(float)0.55, (float)0.50};
    private Table2D flywheel_speed_table = new Table2D(distance, flywheel_speed);
    private Table2D hood_angle_table = new Table2D(distance, hood_angle);
    boolean AutoTargeting;


    /* ---------- Modules & Sensors ---------- */
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    /* ---------- Variables ---------- */
    private double hoodPosition = 0.4; // start in mid position
    private double flywheelRPM;
    private double transferPower;
    private double intakePower;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    int counter;
    private final Pose startPose = new Pose(120, 121, Math.toRadians(35)); // Start Pose of our robot.
    private final Pose score1 = new Pose(96, 96, Math.toRadians(45));
    private final Pose pickupstart = new Pose(96, 53, Math.toRadians(0));
    private final Pose pickupend = new Pose(110, 53, Math.toRadians(0));
    private final Pose score2 = new Pose(96, 96, Math.toRadians(45));
    private final Pose endPose = new Pose(85, 48, Math.toRadians(90));// park Pose of our robot.
    private Follower follower;
    private Path scorePreload;
    private PathChain pickupmotifstart, pickupmotifend, score2path, park;



    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, score1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1.getHeading());

        pickupmotifstart = follower.pathBuilder()
                .addPath(new BezierLine(score1, pickupstart))
                .setLinearHeadingInterpolation(score1.getHeading(), pickupstart.getHeading())
                .build();

        pickupmotifend = follower.pathBuilder()
                .addPath(new BezierLine(pickupstart, pickupend))
                .setLinearHeadingInterpolation(pickupstart.getHeading(), pickupend.getHeading())
                .build();

        score2path = follower.pathBuilder()
                .addPath(new BezierLine(pickupend, score2))
                .setLinearHeadingInterpolation(pickupend.getHeading(), score2.getHeading())
                .build();


        park = follower.pathBuilder()
                .addPath(new BezierLine(score2, endPose))
                .setLinearHeadingInterpolation(score2.getHeading(), endPose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                flywheelRPM = 3500;
                hoodPosition = 0.725;
                follower.followPath(scorePreload);



                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    transferPower = 1.0;
                    intakePower = 1.0;
                    follower.followPath(pickupmotifstart, true);
                    flywheelRPM = 3500;
                    setPathState(2);
                }

                break;


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */




        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        float rpm = 0;
        float angle = 1;
        boolean limelight_available = false;

        Pose2D pose = llModule.limelightResult();
        hood.setPosition(hoodPosition);
        flywheelControl.set_speed((int) flywheelRPM);
        double intakePower = 0.0;
        double transferPower = 0.0;

        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        float limelight_distance = 0;
        if (pose != null) {
            limelight_distance = (float) (1.75*(float) -pose.getX(DistanceUnit.INCH));

            if (limelight_distance < 81 || limelight_distance > 124) {
                limelight_available = true;
            }

            rpm =  (flywheel_speed_table.Lookup(limelight_distance));
            angle = hood_angle_table.Lookup(limelight_distance);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        /* ---------------- LIMELIGHT TELEMETRY ---------------- */


        if (pose != null) {
            telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Rotation (degrees)", pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Limelight", "No valid target");
        }



    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {







        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hood = hardwareMap.get(Servo.class, "hood");





        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.Hardware robot = new Constants.Hardware();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        // Modules
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;

        llModule = new LimelightProcessingModule(limelight, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        flywheel.setDirection(DcMotor.Direction.REVERSE);


    }

}