package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.FCDrivebaseModule;
import org.firstinspires.ftc.teamcode.Modules.IndexerModule;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Modules.Table2D;
import org.firstinspires.ftc.teamcode.Modules.flywheelModule;

@TeleOp(name="H2OLooBots_Final_Bot", group="LinearOpMode")
public class H2OLooBots_Final_Bot extends OpMode {

    /* ---------- Drive Motors ---------- */
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backIntake;
    private DcMotor frontIntake;
    private DcMotor turretRotation;
    private DcMotor flywheel;
    private Servo ball1;
    private Servo ball2;
    private Servo ball3;
    private Servo hood;
    private Servo linearServo;
    private Servo light1;
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
   // private Servo full_light;

    /* ---------- Variables ---------- */
    private double hoodPosition = 1; // start with hood down
    private double flywheelRPM;
    private double turretRotaTION;
    FCDrivebaseModule drivebase;
    private float[] distance = {22, 30, 35, 40,44,52,56,69,81,125,126};
    private float[] flywheel_speed = {2700, 3000, 3000, 3000, 3300, 3200, 3300, 3500, 3350, 4000,4000};
    private float[] hood_angle = { (float)0.67, (float)0.4, (float)0.4, (float)0.4, (float)0.2,(float)0.0,(float)0.0,(float)0.0,(float)0.65,(float)0.55, (float)0.50};
    private Table2D flywheel_speed_table = new Table2D(distance, flywheel_speed);
    private Table2D hood_angle_table = new Table2D(distance, hood_angle);
    boolean AutoTargeting;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");  // ORANGE & port 1 on EXPANSION hub
        backRight = hardwareMap.get(DcMotor.class, "backRight"); // GREEN & port 2 on CONTROL hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //BLUE & port 1 on CONTROL hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");  // RED  & port 0 on EXPANSION hub
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

        drivebase = new FCDrivebaseModule(backLeft, backRight, frontLeft, frontRight, pinpoint);


        // Mecanum motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

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

    @Override
    public void loop() {

//        if(hoodPosition <= .4){
//            hoodPosition = .4;
//        }

        float rpm = 0;
        float angle = 1;
        boolean limelight_available = false;

        Pose2D pose = llModule.limelightResult();
        float limelight_distance = 0;
       // Pose2D pose = null; not sure why we had this but ill just comment it out

        if (pose != null) {
            limelight_distance = (float) (1.75*(float) -pose.getX(DistanceUnit.INCH));

            if (limelight_distance < 81 || limelight_distance > 110) {
                limelight_available = true;
            }

            if(limelight_available){
                rpm =  (flywheel_speed_table.Lookup(limelight_distance));
                angle = hood_angle_table.Lookup(limelight_distance);
            }
            else if (hood_angle_table.Lookup(limelight_distance) <= .45) {
                angle = .45F;
            }
        }

        /* ---------------- DRIVE CODE ---------------- */
        pinpoint.update();
        drivebase.update_Drive(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.options) {
            pinpoint.update();
            pinpoint.resetPosAndIMU();
        }

        flywheelRPM += gamepad2.right_trigger * 50;
        flywheelRPM -= gamepad2.left_trigger * 50;

        flywheelRPM = Math.max(0, Math.min(4200, flywheelRPM));
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- BALL / INTAKE / TRANSFER CONTROL ---------------- */
        double frontintakePower = 0.0;
        double backintakePower = 0.0;

        // --- Touchpad reverses both while held ---
        if (gamepad2.touchpad || gamepad1.touchpad) {
            flywheelRPM = 3500;
            hoodPosition = 0;
        }

        if (gamepad1.left_bumper) {
            frontintakePower = 1;
            backintakePower = 1;
        }
        else {
            frontintakePower = 0;
            backintakePower = 0;
        }

        if (gamepad1.right_bumper) {
            frontintakePower = -1;
            backintakePower = -1;
        }
        else {
            frontintakePower = 0;
            backintakePower = 0;
        }

        // --- Indexer controls ---
        indexerModule.update();
        if (gamepad2.bWasPressed() || gamepad1.bWasPressed() && flywheelRPM >= 3000)
        {
            indexerModule.shootGreen();
        } else if (gamepad2.xWasPressed()||gamepad1.xWasPressed() && flywheelRPM >= 3000)
        {
            indexerModule.shootPurple();
        } else if (gamepad2.yWasPressed()||gamepad1.yWasPressed() && flywheelRPM >=3000)
        {
            indexerModule.shootAll();
        }

        // --- Flywheel Stop (A) ---
        if (gamepad2.a || gamepad1.a) {
            flywheelRPM = 0;
        }

        // Apply powers
        frontIntake.setPower(frontintakePower);
        backIntake.setPower(backintakePower);
        flywheelControl.set_speed((int) flywheelRPM);

        telemetry.addData("front Power", frontintakePower);
        telemetry.addData("back Power", backintakePower);

//        if (gamepad2.dpad_up) {
//            hoodPosition -= 0.005;
//        } else if (gamepad2.dpad_down) {
//            hoodPosition += 0.005;
//        }
        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        if(gamepad2.dpadDownWasPressed() || gamepad1.dpadDownWasPressed()){
            AutoTargeting = !AutoTargeting;
        }

        if(AutoTargeting) {
            flywheelRPM = rpm;
            hoodPosition = angle;
        }

//        if (gamepad2.dpad_left|| gamepad1.dpad_left) {
//            hoodPosition = 0.725;
//            flywheelRPM = 3500;
//        }
//
//        if (gamepad2.dpad_right||gamepad1.dpad_right) {
//            hoodPosition = 0.575;
//            flywheelRPM = 3750;
//        }

        hood.setPosition(hoodPosition);
        flywheelControl.set_speed((int) flywheelRPM);


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
        telemetry.addData("PID Error", flywheelControl.pid_controller.error);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Feedforward", flywheelControl.feedforward_power);
        telemetry.addData("PID Power", flywheelControl.pid_power);
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.addData("AutoTargeting",AutoTargeting);
        telemetry.update();
    }
}