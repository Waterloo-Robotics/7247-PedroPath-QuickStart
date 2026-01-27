package org.firstinspires.ftc.teamcode.test_opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Velocity Testing", group="TestOpMode")
public class shooterprototypeTesting extends OpMode {

    private DcMotor motor;
    private int motor_speed = 0; // 0 = off, 1-4 = power levels
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;


    @Override
    public void init() {
        // start of hw map
        motor = hardwareMap.get(DcMotor.class, "motor");
        // end of hw map
    }

    @Override
    public void start() {
        motor.setPower(0.5);

    }

    @Override
    public void loop() {
        // start of manual velocity control
        boolean rightBumperPressed = gamepad1.right_bumper;
        boolean leftBumperPressed = gamepad1.left_bumper;

        // Right bumper: increase speed
        if (rightBumperPressed && !prevRightBumper) {
            if (motor_speed < 4) {
                motor_speed++;
            }
        }

        if (leftBumperPressed && !prevLeftBumper) {
            motor_speed = 0;
        }

        double power = 0.0;
        switch (motor_speed) {
            case 1:
                power = 0.25;
                break;
            case 2:
                power = 0.5;
                break;
            case 3:
                power = 0.75;
                break;
            case 4:
                power = 1.0;
                break;
            default:
                power = 0.0;
                break;


        }
        prevRightBumper = rightBumperPressed;
        prevLeftBumper = leftBumperPressed;
        // end of motor velocity control
        // telem
        telemetry.addData("Motor Speed Level", motor_speed);
        telemetry.update();

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
