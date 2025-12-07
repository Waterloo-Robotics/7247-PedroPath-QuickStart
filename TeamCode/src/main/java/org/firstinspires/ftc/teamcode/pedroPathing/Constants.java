package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Table2D;
import org.firstinspires.ftc.teamcode.flywheelModule;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-39.68)
            .lateralZeroPowerAcceleration(-68.30)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.01,0))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.1,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,0,0.6,0))
            .centripetalScaling(0.001);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.REVERSE)
            .yVelocity(65.23)
            .xVelocity(78.32);

    public static PinpointConstants localizerConstants = new PinpointConstants()
                    .forwardPodY(3.937)
                    .strafePodX(2.3622)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static class Hardware {
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

        public void init(HardwareMap hardwareMap) {
            flywheel = hardwareMap.get(DcMotor.class, "flywheel");
            intake = hardwareMap.get(DcMotor.class, "intake");
            transfer = hardwareMap.get(DcMotor.class, "transfer");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            hood = hardwareMap.get(Servo.class, "hood");


            flywheel.setDirection(DcMotor.Direction.REVERSE);
        }

        /* ---------- Modules & Sensors ---------- */
        private flywheelModule flywheelControl;
        private Limelight3A limelight;
        private LimelightProcessingModule llModule;

        /* ---------- Variables ---------- */
        private double hoodPosition = 0.4; // start in mid position
        private double flywheelRPM;





    }


}
