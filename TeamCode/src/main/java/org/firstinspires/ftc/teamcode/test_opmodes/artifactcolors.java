package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pocket Color Test", group = "Test")
public class artifactcolors extends OpMode {

    private RevColorSensorV3 pocket1A, pocket1B;
    private RevColorSensorV3 pocket2A, pocket2B;
    private RevColorSensorV3 pocket3A, pocket3B;

    @Override
    public void init() {
        // Initialize sensors
        pocket1A = hardwareMap.get(RevColorSensorV3.class, "pocket1A");
        pocket1B = hardwareMap.get(RevColorSensorV3.class, "pocket1B");
        pocket2A = hardwareMap.get(RevColorSensorV3.class, "pocket2A");
        pocket2B = hardwareMap.get(RevColorSensorV3.class, "pocket2B");
        pocket3A = hardwareMap.get(RevColorSensorV3.class, "pocket3A");
        pocket3B = hardwareMap.get(RevColorSensorV3.class, "pocket3B");

        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addLine("=== POCKET CONTENTS ===");

        telemetry.addData("Pocket 1", detectColor(pocket1A, pocket1B));
        telemetry.addData("Pocket 2", detectColor(pocket2A, pocket2B));
        telemetry.addData("Pocket 3", detectColor(pocket3A, pocket3B));
        telemetry.update();
    }

    // this part was given to me from discord ftc people so dont credit me for it
    private String detectColor(RevColorSensorV3 sensorA, RevColorSensorV3 sensorB) {
        int red   = (sensorA.red()   + sensorB.red())   / 2;
        int green = (sensorA.green() + sensorB.green()) / 2;
        int blue  = (sensorA.blue()  + sensorB.blue())  / 2;

        // Use distance to detect if a ball is present
        double avgDistance = (sensorA.getDistance(DistanceUnit.CM) +
                sensorB.getDistance(DistanceUnit.CM)) / 2.0;

        // Adjust this threshold based on your pocket setup (test with ball in/out)
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