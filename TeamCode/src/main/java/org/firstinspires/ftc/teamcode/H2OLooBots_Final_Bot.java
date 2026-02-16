package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.FCDrivebaseModule;
import org.firstinspires.ftc.teamcode.Modules.IndexerModule;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.Modules.Table2D;
import org.firstinspires.ftc.teamcode.Modules.TurretModule;
import org.firstinspires.ftc.teamcode.Modules.flywheelModule;

@TeleOp(name="H2OLooBots_Final_Bot", group="LinearOpMode")
public class H2OLooBots_Final_Bot extends WlooOpmode {

    private flywheelModule flywheelControl;
    private LimelightProcessingModule llModule;
    private IndexerModule indexerModule;
    private TurretModule turretModule;

    /* ---------- Variables ---------- */
    private double hoodPosition = 1; // start with hood down
    private double flywheelRPM;
    private double turretRotaTION;
    FCDrivebaseModule drivebase;
    private Table2D flywheel_speed_table = new Table2D(WlooConstants.flywheel_distance, WlooConstants.flywheel_speed);
    private Table2D hood_angle_table = new Table2D(WlooConstants.flywheel_distance, WlooConstants.hood_angle);
    boolean AutoTargeting;

    @Override
    public void init() {
        super.init();

        drivebase = new FCDrivebaseModule(backLeft, backRight, frontLeft, frontRight, pinpoint);
        turretModule = new TurretModule(linearServo, turretRotation);

        // Modules
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;

        llModule = new LimelightProcessingModule(limelight, telemetry);
        limelight.start();

        indexerModule = new IndexerModule(ball1, color1a, color1b, ball2, color2a, color2b, ball3, color3a, color3b, light1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad2.aWasPressed())
        {
            turretModule.home_turret();
        } else if (gamepad2.dpadDownWasPressed())
        {
            turretModule.go_backwards();
        } else if (gamepad2.dpadUpWasPressed())
        {
            turretModule.go_forwards();
        }else if(gamepad2.dpadLeftWasPressed())
        {
            turretModule.go_left();
        }else if(gamepad2.dpadRightWasPressed())
        {
            turretModule.go_right();
        }
        turretModule.update();
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
            else{
                rpm = 2500;
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
        else if(gamepad1.right_bumper){
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

        if(gamepad1.dpadDownWasPressed()){
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
        telemetry.addData("Artifacts Deletected", indexerModule.num_artifacts);
        telemetry.update();
    }
}