package org.firstinspires.ftc.teamcode.test_opmodes;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.AngleModule;
import org.firstinspires.ftc.teamcode.Modules.FieldPositionEstimation;
@TeleOp(name=" Angle Module Test", group="TestOpMode")
public class AngleModuleTest extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private FieldPositionEstimation field_estimator;
    private GoBildaPinpointDriver pinpoint;
    private AngleModule angleController;
    private double desired_angle;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        field_estimator = new FieldPositionEstimation(pinpoint, false);
        angleController = new AngleModule();

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);


    }
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            field_estimator.reset_pinpoint();
            desired_angle = 0;
        }
        field_estimator.update_from_pinpoint();

        double turn = 0;
        desired_angle = desired_angle + gamepad1.right_stick_x * 1.4;

        if(desired_angle > 180){
            desired_angle = desired_angle- 360;
        } else if (desired_angle <-180) {
            desired_angle = desired_angle + 360;
        }

        if (gamepad1.b){
            turn = -angleController.set_Angle(desired_angle, field_estimator.relative_robot_position.getHeading(AngleUnit.DEGREES));
        }

        double y = -gamepad1.left_stick_y * 0.6;   // Forward/backward
        double x = gamepad1.left_stick_x * 0.6;    // Strafe left/right

        // Calculate motor powers for mecanum drive
        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        // Normalize motor powers to stay within [-1.0, 1.0]
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }


        // Set powers to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("Desired Angle", desired_angle);
        telemetry.addData("Error", angleController.pid_controller.error);
        telemetry.addData("Rotation", "%f.2", field_estimator.relative_robot_position.getHeading(AngleUnit.DEGREES));
        angleController.pid_controller.add_telemetry(telemetry);
        telemetry.update();
    }}

