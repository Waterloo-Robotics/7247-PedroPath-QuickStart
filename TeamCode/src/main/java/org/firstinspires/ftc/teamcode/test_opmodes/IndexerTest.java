package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Indexer Test", group="TestOpMode")
public class IndexerTest extends OpMode {

    private Servo SlotO;
    private Servo SlotT;
    private Servo SlotTH;

    @Override
    public void init(){

        SlotO = hardwareMap.get(Servo.class, "ball1");
        SlotT = hardwareMap.get(Servo.class, "ball2");
        SlotTH = hardwareMap.get(Servo.class, "ball3");
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            SlotO.setPosition(0);

        }

        if (gamepad1.a) {
            SlotT.setPosition(.5);

        }

        if (gamepad1.b) {
            SlotTH.setPosition(1);

        }

        if (gamepad1.y) {
            SlotTH.setPosition(0);
            SlotT.setPosition(1);
            SlotO.setPosition(1);

        }
    }
}

