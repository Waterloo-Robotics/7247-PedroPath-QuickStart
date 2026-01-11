package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Table2D;
import org.firstinspires.ftc.teamcode.Modules.flywheelModule;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(1.535)
            .lateralZeroPowerAcceleration(-81.67)
            .forwardZeroPowerAcceleration(-43.44)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.031, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9,0,0.3,0.15))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.25,0,0.00001,0.6,0.01))
            .centripetalScaling(0.0000001);

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
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(72.27)
            .yVelocity(55.33);

    public static PinpointConstants localizerConstants = new PinpointConstants()
                    .forwardPodY(6.6318)
                    .strafePodX(0)
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
