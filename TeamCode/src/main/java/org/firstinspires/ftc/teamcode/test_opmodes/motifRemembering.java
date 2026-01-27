package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class motifRemembering extends OpMode {

    private Limelight3A limelight;

    // variable to remember the first motif tag (21, 22, or 23) seen
    private Integer rememberedMotifTag = null;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized - Waiting for motif tag");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    int id = tag.getFiducialId();
                    telemetry.addData("Detected Tag ID", id);
                }
                if (rememberedMotifTag == null) {
                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        int id = tag.getFiducialId();
                        if (id == 21 || id == 22 || id == 23) {
                            rememberedMotifTag = id;
                            telemetry.addData("MOTIF TAG REMEMBERED", rememberedMotifTag);
                            break;
                        }
                    }
                }
            } else {
                telemetry.addData("AprilTags", "None detected");
            }

            // Always display the currently remembered motif tag
            if (rememberedMotifTag != null) {
                telemetry.addData("Remembered Motif Tag", rememberedMotifTag);
            } else {
                telemetry.addData("Remembered Motif Tag", "None yet");
            }

        } else {
            telemetry.addData("Limelight", "No valid result");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}