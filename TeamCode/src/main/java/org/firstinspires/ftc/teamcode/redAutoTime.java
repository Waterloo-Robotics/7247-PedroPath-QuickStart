package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "redAutoTime", group = "Auto")
public class redAutoTime extends LinearOpMode {

    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private DcMotorEx flywheel;
    private DcMotor intake, transfer;
    private Servo hood;

    @Override
    public void runOpMode() {

        // Hardware mapping
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");

        // Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            // LOWER hood
            hood.setPosition(0.7);
            sleep(300);

            // Drive backward
            frontLeft.setPower(-0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(-0.4);
            backRight.setPower(-0.4);
            sleep(500);

            // Stop drive
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // ----------------------------
            int targetRPM = 2650;
            double ticksPerRev = 28;
            double targetVelocity = (targetRPM / 60.0) * ticksPerRev;

            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setVelocity(targetVelocity);  // << actual speed control

            sleep(2000); // allow time to reach speed

            // Run intake/transfer
            intake.setPower(1.0);
            transfer.setPower(-1.0);
            sleep(1000);

            intake.setPower(1.0);
            transfer.setPower(-1.0);
            sleep(1000);

            intake.setPower(1.0);
            transfer.setPower(-1.0);
            sleep(1000);

            // Stop intake/transfer
            intake.setPower(0);
            transfer.setPower(0);

            // Extra flywheel time
            sleep(1000);

            // Stop flywheel
            flywheel.setVelocity(0);

            sleep(500);
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);

            sleep(500);
            frontLeft.setPower(-0.3);
            frontRight.setPower(-0.3);
            backLeft.setPower(-0.3);
            backRight.setPower(-0.3);
            sleep(1000);

        }
    }
}