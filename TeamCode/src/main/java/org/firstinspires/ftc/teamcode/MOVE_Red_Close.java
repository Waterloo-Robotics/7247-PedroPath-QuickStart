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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.IndexerModule;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Modules.Table2D;
import org.firstinspires.ftc.teamcode.Modules.flywheelModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "MOVE RED CLOSE", group = "Examples")
public class MOVE_Red_Close extends WlooOpmode {


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
    private Servo light1;
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;
    private IndexerModule indexerModule;

    private Table2D flywheel_speed_table = new Table2D(WlooConstants.flywheel_distance, WlooConstants.flywheel_speed);
    private Table2D hood_angle_table = new Table2D(WlooConstants.flywheel_distance, WlooConstants.hood_angle);
    boolean AutoTargeting;
    GoBildaPinpointDriver pinpoint;



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
    private final Pose end = new Pose(96, 24, Math.toRadians(180));  // shooting first row




    public void setCallbackran(){
        callbackran = true;
    }


    private Follower follower;
    private Path shoot1Path;
    private PathChain pickup1startPath,pickup1endPath,shoot2stallPath,shoot2Path,pickup2startPath,pickup2endPath,shoot3stallPath,shoot3Path,pickup3startPath,pickup3endPath,shoot4Path,endPath;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        shoot1Path = new Path(new BezierLine(startPose, end));
        shoot1Path.setLinearHeadingInterpolation(startPose.getHeading(), end.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                follower.followPath(shoot1Path);
                setPathState(0);
                break;
            case 0:

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
        flywheelControl.set_speed((int) flywheelRPM);

//        float rpm = 0;
//        float angle = 1;
//        boolean limelight_available = false;
//
//        Pose2D pose = llModule.limelightResult();
//        hood.setPosition(hoodPosition);
//        flywheelControl.set_speed((int) flywheelRPM);
//
//
//        float limelight_distance = 0;
//        if (pose != null) {
//            limelight_distance = (float) (1.75*(float) -pose.getX(DistanceUnit.INCH));
//
//            if (limelight_distance < 81 || limelight_distance > 124) {
//                limelight_available = true;
//            }
//
//            rpm =  (flywheel_speed_table.Lookup(limelight_distance));
//            angle = hood_angle_table.Lookup(limelight_distance);
//        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        /* ---------------- LIMELIGHT TELEMETRY ---------------- */


//        if (pose != null) {
//            telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
//            telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
//            telemetry.addData("Rotation (degrees)", pose.getHeading(AngleUnit.DEGREES));
//        } else {
//            telemetry.addData("Limelight", "No valid target");
//        }

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
        light1 = hardwareMap.get(Servo.class, "light1");


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

        ((LynxI2cDeviceSynch) color1a.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color1b.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color2a.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color2b.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color3a.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) color3b.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}