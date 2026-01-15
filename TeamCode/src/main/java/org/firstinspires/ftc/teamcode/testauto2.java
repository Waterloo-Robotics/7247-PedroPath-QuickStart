package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.IndexerModule;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Modules.Table2D;
import org.firstinspires.ftc.teamcode.Modules.flywheelModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "test path for red far", group = "Examples")
public class testauto2 extends OpMode {
    private DcMotor backIntake;
    private DcMotor frontIntake;
    private DcMotor turretRotation;
    private DcMotor flywheel;
    private Servo ball1;
    private Servo ball2;
    private Servo ball3;
    private Servo hood;
    private Servo linearServo;
    private RevColorSensorV3 color1a;
    private RevColorSensorV3 color1b;
    private RevColorSensorV3 color2a;
    private RevColorSensorV3 color2b;
    private RevColorSensorV3 color3a;
    private RevColorSensorV3 color3b;
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;
    private IndexerModule indexerModule;
    private float[] distance = {22, 30, 35, 40,44,52,56,69,81,125,126};
    private float[] flywheel_speed = {2700, 3000, 3000, 3000, 3300, 3200, 3300, 3500, 3350, 4000,4000};
    private float[] hood_angle = { (float)0.67, (float)0.4, (float)0.4, (float)0.4, (float)0.2,(float)0.0,(float)0.0,(float)0.0,(float)0.65,(float)0.55, (float)0.50};
    private org.firstinspires.ftc.teamcode.Modules.Table2D flywheel_speed_table = new org.firstinspires.ftc.teamcode.Modules.Table2D(distance, flywheel_speed);
    private org.firstinspires.ftc.teamcode.Modules.Table2D hood_angle_table = new Table2D(distance, hood_angle);
    boolean AutoTargeting;
    GoBildaPinpointDriver pinpoint;


    /* ---------- Variables ---------- */
    private double hoodPosition;
    private double flywheelRPM;
    double frontintakePower = 0.0;
    double backintakePower = 0.0;
    private boolean callbackran = false;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    int counter;
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose shoot1 = new Pose(83, 83, Math.toRadians(45));  // shooting preload
    private final Pose pickup1start = new Pose(104, 35, Math.toRadians(0));  // pick up 1st row start
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
        flywheelRPM = 3500;
    }

    public void flywheel_off(){
        flywheelRPM = 0;
    }

    public void IntakeON(){
        frontintakePower = 1;
        backintakePower = 1;
    }

    public void IntakeOFF(){
        frontintakePower = 0;
        backintakePower = 0;
        indexerModule.shootAll();
    }
    public void shooALL(){
        indexerModule.shootAll();
    }
    public void shooGREEN(){
        indexerModule.shootGreen();
    }
    public void shootPURPLE(){
        indexerModule.shootPurple();
    }

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
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(shoot1Path);

                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1startPath);

                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1endPath);

                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(shoot2stallPath);

                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(shoot2Path);

                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2startPath);

                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2endPath);

                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(shoot3stallPath);

                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(shoot3Path);

                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(pickup3startPath);

                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(pickup3endPath);

                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(shoot4Path);

                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(endPath);

                    setPathState(13);
                }
                break;


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
        telemetry.update();



    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {


        backIntake= hardwareMap.get(DcMotor.class, "backIntake"); // YELLOW & port 2 on EXPANSION hub
        frontIntake= hardwareMap.get(DcMotor.class, "frontIntake"); // PURPLE & port 0 on CONTROL
        flywheel = hardwareMap.get(DcMotor.class, "flywheel"); // WHITE & port 3 onCONTROL hub
        turretRotation = hardwareMap.get(DcMotor.class, "turretRotation"); // GREY & port 3 on EXPANSION hub
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"); // 12c Bus 0 on EXPANSION hub
        ball1 = hardwareMap.get(Servo.class, "ball1"); // RED servo port 0 on EXPANSION hub
        ball2 = hardwareMap.get(Servo.class, "ball2"); // YELLOW servo port 5 on CONTROL hub
        ball3 = hardwareMap.get(Servo.class, "ball3"); // ORANGE servo port 0  on CONTROL hub
        linearServo = hardwareMap.get(Servo.class, "linearServo"); // GREEN servo port 1 on EXPANSION hub
        hood = hardwareMap.get(Servo.class, "hood"); // BLUE & servo port 1 on CONTROL hub
        color1a = hardwareMap.get(RevColorSensorV3.class, "color1a"); // BLUE & 12c Bus 3 on EXPANSION hub
        color1b = hardwareMap.get(RevColorSensorV3.class, "color1b"); // PURPLE & 12c Bus 2 on EXPANSION hub
        color2a = hardwareMap.get(RevColorSensorV3.class, "color2a"); // YELLOW & 12c Bus 3 on CONTROL hub
        color2b = hardwareMap.get(RevColorSensorV3.class, "color2b"); // GREEN & 12c Bus 2 on CONTROL hub
        color3a = hardwareMap.get(RevColorSensorV3.class, "color3a"); // ORANGE & 12c Bus 1 on CONTROL hub
        color3b = hardwareMap.get(RevColorSensorV3.class, "color3b"); // RED & 12c Bus 0 on CONTROL hub
        limelight = hardwareMap.get(Limelight3A.class, "limelight");





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
        limelight.start();

        ((LynxI2cDeviceSynch) color1a.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color1b.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color2a.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color2b.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color3a.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color3b.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        indexerModule = new IndexerModule(ball1, color1a, color1b, ball2, color2a, color2b, ball3, color3a, color3b);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


    }

}