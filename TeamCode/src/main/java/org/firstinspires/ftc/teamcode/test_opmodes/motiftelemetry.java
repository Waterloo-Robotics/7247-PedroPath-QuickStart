package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;

@TeleOp(name = "Motif Telemetry")
public class motiftelemetry extends OpMode {

    // motif booleans
    private boolean id23;
    //ppg
    private boolean id22;
    // pgp
    private boolean id21;
    private byte scannedID;
    // gpp

    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llModule = new LimelightProcessingModule(limelight, telemetry);
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        Pose2D pose = llModule.limelightResult();
        limelight.getLatestResult();

            if (id23 == true) {
                scannedID = 23;
            } else if (id22 == true) {
                scannedID = 22;
            } else if (id21 == true) {
                scannedID = 21;
            }

            if (pose != null) {
                telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Rotation (degrees)", pose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addData("Limelight", "No valid target");
            }

            telemetry.update();
        }
    }


