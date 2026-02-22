package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.IndexerModule;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Modules.Table2D;
import org.firstinspires.ftc.teamcode.Modules.flywheelModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SCORE RED CLOSE", group = "Examples")
public class Score_Red_Close extends WlooOpmode {

    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;
    private IndexerModule indexerModule;

    private Table2D flywheel_speed_table = new Table2D(WlooConstants.flywheel_distance, WlooConstants.flywheel_speed);
    private Table2D hood_angle_table = new Table2D(WlooConstants.flywheel_distance, WlooConstants.hood_angle);
    boolean AutoTargeting;


    /* ---------- Variables ---------- */
    private double hoodPosition;
    private double flywheelRPM;
    double frontintakePower;
    double backintakePower;
    private boolean callbackran = false;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = -1;
    int counter;
    private final Pose startPose = new Pose(121, 120, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose shoot1 = new Pose(108, 108, Math.toRadians(225));  // shooting preload
    private final Pose pickup1start = new Pose(96, 125, Math.toRadians(360));  // pick up 1st row start
    private final Pose pickup1end = new Pose(129, 35, Math.toRadians(0));  // picking up 1st row end
    private final Pose shoot2stall = new Pose(85, 50, Math.toRadians(0));  // shooting preload
    private final Pose shoot2 = new Pose(83, 83, Math.toRadians(45));  // shooting first row
    private final Pose pickup2start = new Pose(104, 60, Math.toRadians(0));  // pick up 1st row start
    private final Pose pickup2end = new Pose(129, 60, Math.toRadians(0));  // picking up 1st row end
    private final Pose shoot3stall = new Pose(84, 69, Math.toRadians(45));  // shooting preload
    private final Pose shoot3 = new Pose(83, 83, Math.toRadians(45));  // shooting first row
    private final Pose pickup3start = new Pose(104, 83, Math.toRadians(0));  // pick up 1st row start
    private final Pose pickup3end = new Pose(129, 83, Math.toRadians(0));  // picking up 1st row end
    private final Pose shoot4 = new Pose(83, 83, Math.toRadians(45));  // shooting first row
    private final Pose end = new Pose(96, 24, Math.toRadians(45));  // shooting first row




    public void flywheel_on(){
        flywheelRPM = -2800;
    }
    public void flywheel_off(){
        flywheelRPM = 0;
    }
    public void intake_on(){
        frontintakePower = 1;
    }
    public void shooALL(){
        indexerModule.shootAll();
    }
    //    public void shooGREEN(){
//        indexerModule.shootGreen();
//    }
//    public void shootPURPLE(){
//        indexerModule.shootPurple();
//    }
    public void hoodUP() {hoodPosition = 0.782f; };
    public void setCallbackran(){
        callbackran = true;
    }


    private Follower follower;
    private Path shoot1Path;
    private PathChain pickup1startPath,pickup1endPath,shoot2stallPath,shoot2Path,pickup2startPath,pickup2endPath,shoot3stallPath,shoot3Path,pickup3startPath,pickup3endPath,shoot4Path,endPath;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        shoot1Path = new Path(new BezierLine(startPose, shoot1));
        shoot1Path.setLinearHeadingInterpolation(startPose.getHeading(), shoot1.getHeading());

        pickup1startPath = follower.pathBuilder()
                .addPath(new BezierLine(shoot1, pickup1start))
                .setLinearHeadingInterpolation(shoot1.getHeading(), pickup1start.getHeading())
                .build();
        pickup1endPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup1start, pickup1end))
                .setLinearHeadingInterpolation(pickup1start.getHeading(), pickup1end.getHeading())
                .build();
        shoot2stallPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup1end, shoot2stall))
                .setLinearHeadingInterpolation(pickup1end.getHeading(), shoot2stall.getHeading())
                .build();
        shoot2Path = follower.pathBuilder()
                .addPath(new BezierLine(shoot2stall, shoot2))
                .setLinearHeadingInterpolation(shoot2stall.getHeading(), shoot2.getHeading())
                .build();
        pickup2startPath = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, pickup2start))
                .setLinearHeadingInterpolation(shoot2.getHeading(), pickup2start.getHeading())
                .build();
        pickup2endPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup2start, pickup2end))
                .setLinearHeadingInterpolation(pickup2start.getHeading(), pickup2end.getHeading())
                .build();
        shoot3stallPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup2end, shoot3stall))
                .setLinearHeadingInterpolation(pickup2end.getHeading(), shoot3stall.getHeading())
                .build();
        shoot3Path = follower.pathBuilder()
                .addPath(new BezierLine(shoot3stall, shoot3))
                .setLinearHeadingInterpolation(shoot3stall.getHeading(), shoot3.getHeading())
                .build();
        pickup3startPath = follower.pathBuilder()
                .addPath(new BezierLine(shoot3, pickup3start))
                .setLinearHeadingInterpolation(shoot3.getHeading(), pickup3start.getHeading())
                .build();
        pickup3endPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup3start, pickup3end))
                .setLinearHeadingInterpolation(pickup3start.getHeading(), pickup3end.getHeading())
                .build();
        shoot4Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup3end, shoot4))
                .setLinearHeadingInterpolation(pickup3end.getHeading(), shoot4.getHeading())
                .build();
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(shoot4, end))
                .setLinearHeadingInterpolation(shoot4.getHeading(), end.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                follower.followPath(shoot1Path);
                hoodUP();
                flywheel_on();
                setPathState(0);
                break;
            case 0:
                if(counter > 30){
                    indexerModule.shootAll();
                    setPathState(1);
                }
                break;
            case 1:
                if(counter > 50){
                    indexerModule.shootAll();
                    setPathState(2);
                }

                break;
            case 2:
                if(counter > 75){
                    indexerModule.shootAll();
                    setPathState(3);
                }

                break;
            case 3:
                if (!follower.isBusy() && counter > 100) {
                    follower.followPath(pickup1startPath);
                    flywheel_off();
                    setPathState(4);
                }
                break;
//
//            case 2:
//                if (!follower.isBusy()  /*&& counter > */ ) {
//                    follower.followPath(pickup1endPath);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(shoot2stallPath);
//
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(shoot2Path);
//
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(/*counter > 50*/){
//                    indexerModule.shootAll();
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()  /*&& counter > */) {
//                    follower.followPath(pickup2startPath);
//                    frontintakePower = 1;
//
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//                    follower.followPath(pickup2endPath);
//                    frontintakePower = 0;
//
//
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    follower.followPath(shoot3stallPath);
//
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy()) {
//                    follower.followPath(shoot3Path);
//
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                if(/*counter > 50*/){
//                    indexerModule.shootAll();
//                    setPathState(11);
//                }
//
//                break;
//            case 11:
//                if (!follower.isBusy()  /*&& counter > */ ) {
//                    follower.followPath(pickup3startPath);
//                    frontintakePower = 1;
//
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if (!follower.isBusy()) {
//                    follower.followPath(pickup3endPath);
//                    frontintakePower = 0;
//
//
//                    setPathState(13);
//                }
//                break;
//            case 13:
//                if (!follower.isBusy()) {
//                    follower.followPath(shoot4Path);
//
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if(/*counter > 50*/){
//                    indexerModule.shootAll();
//                    setPathState(16);
//                }
//                break;
//            case 15:
//                if (!follower.isBusy()  /*&& counter > */ ) {
//                    follower.followPath(endPath);
//                    flywheel_off()
//                    frontintakePower = 0;
//
//                    setPathState(16);
//                }
//                break;


        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        counter = counter + 1 ;
        indexerModule.update();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        frontIntake.setPower(frontintakePower);
        autonomousPathUpdate();

        float rpm = 0;
        float angle = 1;
        boolean limelight_available = false;

        Pose2D pose = llModule.limelightResult();
        hood.setPosition(hoodPosition);
        flywheelControl.set_speed((int) flywheelRPM);


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
        /* ---------------- LIMELIGHT TELEMETRY ---------------- */


        if (pose != null) {
            telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Rotation (degrees)", pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Limelight", "No valid target");
        }

        /* ---------------- GENERAL TELEMETRY ---------------- */
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.addData("callback run", callbackran);
        telemetry.addData("Timer", counter);

        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();

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

        indexerModule = new IndexerModule(ball1, color1a, color1b, ball2, color2a, color2b, ball3, color3a, color3b, light1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}