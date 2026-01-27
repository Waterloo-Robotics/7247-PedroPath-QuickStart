package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Artifact Launch", group = "Test")
public class artifactlaunch extends OpMode {

    private RevColorSensorV3 pocket1A, pocket1B;
    private RevColorSensorV3 pocket2A, pocket2B;
    private RevColorSensorV3 pocket3A, pocket3B;

    private Servo servo1, servo2, servo3;

    private List<Integer> availablePurples = new ArrayList<>();
    private List<Integer> availableGreens = new ArrayList<>();

    private boolean isLaunching = false;
    private double launchEndTime = 0;
    private static final double LAUNCH_DURATION = 0.4; // how long the servo stays in launch pos

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void init() {
        // Sensors
        pocket1A = hardwareMap.get(RevColorSensorV3.class, "color1a");
        pocket1B = hardwareMap.get(RevColorSensorV3.class, "color1b");
        pocket2A = hardwareMap.get(RevColorSensorV3.class, "color2a");
        pocket2B = hardwareMap.get(RevColorSensorV3.class, "color2b");
        pocket3A = hardwareMap.get(RevColorSensorV3.class, "color3a");
        pocket3B = hardwareMap.get(RevColorSensorV3.class, "color3b");

        // Servos
        servo1 = hardwareMap.get(Servo.class, "ball1");
        servo2 = hardwareMap.get(Servo.class, "ball2");
        servo3 = hardwareMap.get(Servo.class, "ball3");

        // rest pos (probably need to adjust later)
        servo1.setPosition(1.0);
        servo2.setPosition(1.0);
        servo3.setPosition(0);

        telemetry.update();
    }

    @Override
    public void loop() {
        // update dem balls
        updateAvailableBalls();

        // only take input if not launching (i think it works)
        boolean currDpadUp = gamepad1.dpad_up;
        if (currDpadUp && !prevDpadUp && !isLaunching) {
            launchNext("PURPLE");
        }
        prevDpadUp = currDpadUp;

        boolean currDpadDown = gamepad1.dpad_down;
        if (currDpadDown && !prevDpadDown && !isLaunching) {
            launchNext("GREEN");
        }
        prevDpadDown = currDpadDown;

        // return servo to hold position after duration
        if (isLaunching) {
            if (getRuntime() >= launchEndTime) {
                resetAllServos(); // return all servos to rest pos
                isLaunching = false;
            }
        }

        // tea lemon tree
        telemetry.addLine("=== POCKET CONTENTS ===");
        telemetry.addData("Pocket 1", detectColor(pocket1A, pocket1B));
        telemetry.addData("Pocket 2", detectColor(pocket2A, pocket2B));
        telemetry.addData("Pocket 3", detectColor(pocket3A, pocket3B));

        telemetry.addLine();
        telemetry.addData("Available Purples", availablePurples);
        telemetry.addData("Available Greens", availableGreens);
        telemetry.addData("Launching", isLaunching ? "Yes" : "No");
        telemetry.addLine("UP: PURPLE | DOWN: GREEN");
        telemetry.update();
    }

    private void updateAvailableBalls() {
        availablePurples.clear();
        availableGreens.clear();

        String c1 = detectColor(pocket1A, pocket1B);
        String c2 = detectColor(pocket2A, pocket2B);
        String c3 = detectColor(pocket3A, pocket3B);

        if ("PURPLE".equals(c1)) availablePurples.add(1);
        if ("PURPLE".equals(c2)) availablePurples.add(2);
        if ("PURPLE".equals(c3)) availablePurples.add(3);

        if ("GREEN".equals(c1)) availableGreens.add(1);
        if ("GREEN".equals(c2)) availableGreens.add(2);
        if ("GREEN".equals(c3)) availableGreens.add(3);

        // Already naturally sorted since we add in order 1->2->3
    }

    private void launchNext(String color) {
        List<Integer> available = color.equals("PURPLE") ? availablePurples : availableGreens;

        if (available.isEmpty()) {
            // Nothing to launch
            return;
        }

        // Launch the lowest-numbered pocket
        int pocket = available.remove(0); // remove it so it's no longer considered available

        launchPocket(pocket);

        isLaunching = true;
        launchEndTime = getRuntime() + LAUNCH_DURATION;
    }

    private void launchPocket(int pocket) {
        switch (pocket) {

            /////// ughhhhhhh we gotta like test this??? ughhh i hate work why cant we just cast a spell to know the correct servo positions
            // ready to change this later
            case 1:
                servo1.setPosition(0);
                break;
            case 2:
                servo2.setPosition(0);
                break;
            case 3:
                servo3.setPosition(1.0);
                break;
        }
    }

    private void resetAllServos() {
        servo1.setPosition(1.0);
        servo2.setPosition(1.0);
        servo3.setPosition(0.0);
    }

    // Color detection (with big help from ftc discord guys who are much cool)
    private String detectColor(RevColorSensorV3 sensorA, RevColorSensorV3 sensorB) {
        int red   = (sensorA.red()   + sensorB.red())   / 2;
        int green = (sensorA.green() + sensorB.green()) / 2;
        int blue  = (sensorA.blue()  + sensorB.blue())  / 2;

        double avgDistance = (sensorA.getDistance(DistanceUnit.CM) +
                sensorB.getDistance(DistanceUnit.CM)) / 2.0;

        if (avgDistance > 3.0) {
            return "EMPTY";
        }

        if (green > red + 50 && green > blue + 50) {
            return "GREEN";
        } else if (red + blue > green + 80) {
            return "PURPLE";
        } else {
            return "UNKNOWN";
        }
    }
}