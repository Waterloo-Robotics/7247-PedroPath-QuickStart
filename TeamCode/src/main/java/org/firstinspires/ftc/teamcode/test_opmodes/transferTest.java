package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class transferTest extends OpMode {
    private DcMotor transfer;

    public void init() {
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {
        if (gamepad1.aWasPressed()) {
            transfer.setPower(1);
        } else if (gamepad1.bWasPressed())
            transfer.setPower(-1);
        }

    }

