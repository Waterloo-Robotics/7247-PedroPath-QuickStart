package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.flywheelModule;

@TeleOp(name="HoodFly Setpoint Test", group="TestOpMode")
public class HoodFlywheelTest extends OpMode {
    private DcMotor flywheel;
    private Servo hood;
    flywheelModule flywheelControl;
    private double flywheelRPM;


    @Override
    public void init(){
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;
    }
    public void init_loop() {
    }


    @Override
    public void start() {
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_left) {
            hood.setPosition(0.417);
            flywheelRPM = -3500;
        }

        if (gamepad1.a) {
            flywheelRPM = 0;
        }
        flywheelControl.set_speed((int) flywheelRPM);
    }
}

