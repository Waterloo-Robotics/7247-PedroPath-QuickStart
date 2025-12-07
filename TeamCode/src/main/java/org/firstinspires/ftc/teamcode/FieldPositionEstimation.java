package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FieldPositionEstimation {

    private GoBildaPinpointDriver pinpoint;
    private Pose2D field_position;
    public Pose2D relative_robot_position;
    private boolean on_red_side; // what is this for?

    private LimelightProcessingModule limelight;
    private Pose2D limelight_position;

    public FieldPositionEstimation(GoBildaPinpointDriver pinpoint, boolean on_red_side)
    {
        this.pinpoint = pinpoint;
        this.pinpoint.setOffsets(155, -25, DistanceUnit.MM);
        this.on_red_side = on_red_side;
        this.pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    public void attachLimelight(LimelightProcessingModule limelight) {
        this.limelight = limelight;
    }
    public void update_from_pinpoint() {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double rot = pinpoint.getHeading(AngleUnit.DEGREES);
        this.relative_robot_position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, rot);
    }

    public void update_from_limelight() {
        if (this.limelight == null) return;

        Pose2D field_pos = this.limelight.getFieldSpacePose();

        if (field_pos != null ) {
            this.limelight_position = field_pos;
        }
    }

    // Fuse the data from IMU and Limelight
    public Pose2D getFusedFieldPosition() {
        // If Limelight position is available, use it
        if (limelight_position != null) {
            this.field_position = limelight_position;
        }
        // Otherwise, use the relative IMU position
        else if (relative_robot_position != null) {
            this.field_position = relative_robot_position;
        }

        return this.field_position;
    }
    public void reset_pinpoint() {
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }
}
