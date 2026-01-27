package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Stemfest Tele", group="TestOpMode")
public class stemfestTeleop extends LinearOpMode {

    private DcMotor motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Configuration ---
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime();
        double lastTime = timer.seconds();
        double currentPower = 0.0;

        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();
        waitForStart();

        // main loop
        while (opModeIsActive()) {

            currentPower += gamepad1.left_stick_y * 0.01;
            if (gamepad1.dpadDownWasPressed()) {
                currentPower =0;
            }
            if (gamepad1.dpadUpWasPressed()) {
                currentPower =-1;
            }
            if (currentPower < -1) {
                currentPower = -1;
            }
            if (currentPower > 0) {
                currentPower = 0;
            }

            // Apply power and show telemetry
            motor.setPower(currentPower);

            telemetry.clearAll();
            telemetry.addData("Motor Power", "%.3f", currentPower);
            telemetry.update();


            sleep(10); // 10 ms pause (safe small delay)
        }



        // stop motor when opmode ends
        motor.setPower(0.0);
    }
}
