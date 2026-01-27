package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Haptics Test", group="TestOpMode")
public class haptics extends OpMode {


    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // rumbles for a duration of given time in miliseconds
        if (gamepad1.a) {
            gamepad1.rumble(100);
        }

        /*rumble blips is a way you can control the pattern or rumble motors used, but
        we didnt define anything in this code, so gamepad B doesn't do much of a difference
         */
        if (gamepad1.b) {
            gamepad1.rumbleBlips(3);
        }

    }

}
