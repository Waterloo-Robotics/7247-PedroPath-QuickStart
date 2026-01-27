package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.LimelightProcessingModule;

@TeleOp(name = "Limelight test", group="TestOpMode")
public class limelightModuleTest extends OpMode {

    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    @Override
    public void init() {
        // Initialize the Limelight from the hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Create LimelightProcessingModule instance
        llModule = new LimelightProcessingModule(limelight, telemetry);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Get the result from the Limelight
        Pose2D pose = llModule.limelightResult();

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
