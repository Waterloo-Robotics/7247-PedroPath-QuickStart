/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test_opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.flywheelModule;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Hood Test", group="TestOpMode")
public class HoodTest extends OpMode{
 //   private DcMotor backLeft;
 //   private DcMotor backRight;
  //  private DcMotor frontLeft;
 //   private DcMotor frontRight;
    private DcMotor flywheel;
//    private DcMotor intake;
    private Servo hood;
    private double hoodPosition = 0.0;

    /* start of module stuff */
    flywheelModule flywheelControl;
    /* end of module stuff */
    private double flywheelRPM;

// hood control booleans
// end of hood control booleans

    @Override
    public void init() {
        // start of hardware map stuff ----
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");
//        intake = hardwareMap.get(DcMotor.class, "intake");
        // end of hardware map stuff

        // Turret motor
        // Set motor directions for mecanum drive
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);

        // module stuff
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {
        /* start of drive code
        --------------------------*/
    /*    double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = gamepad1.left_stick_x;    // Strafe left/right
        double turn = gamepad1.right_stick_x; // Rotate in place

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
            backRightPower /= max;         *.
        }


        // Set powers to motors
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
        /* end of drive code
        ------------------------- */

        //flywheel
        // flywheelControl.set_speed();// will be changed later when velocity is finished
        flywheelRPM += gamepad1.left_stick_y * 25;

        if (flywheelRPM > 0)
        {
            flywheelRPM = 0;
        }
        else if (flywheelRPM < -4200)
        {
            flywheelRPM = -4200;

        }

        flywheelControl.set_speed((int)flywheelRPM);

        /* start of drive stuff
        ------------------ */

//        if (gamepad1.right_trigger > 0) {
//            intake.setPower(1);
//        }
//

        // start of hood control stuff
        // Control servo with gamepad
        if (gamepad1.dpad_up && hoodPosition < 1.0) {
            hoodPosition += 0.001;

        } else if (gamepad1.dpad_down && hoodPosition > 0.0) {
            hoodPosition -= 0.001;
        }
        hood.setPosition(hoodPosition);
        // end of hood control stuff

        telemetry.addData("PID Error", flywheelControl.pid_controller.error);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Feedforward", flywheelControl.feedforward_power);
        telemetry.addData("PID", flywheelControl.pid_power);
        telemetry.addData("Left Stick", gamepad1.left_stick_y);
        telemetry.addData("Hood Servo", hood.getPosition());
        telemetry.update();
    }

}







