package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FieldPositionEstimation {
    /* Purpose of this file
    * The end goal is to ask this module where we are on the field.
    * It may be using PinPoint or it may be updated with April Tag positions
    * to "re-localize".
    * */
    private GoBildaPinpointDriver pinpoint;
    private Pose2D field_position;
    public Pose2D relative_robot_position;
    private boolean on_red_side;

    public FieldPositionEstimation(GoBildaPinpointDriver pinpoint, boolean on_red_side)
    {
        this.pinpoint = pinpoint;
        this.pinpoint.setOffsets(155, -25, DistanceUnit.MM);
        this.on_red_side = on_red_side;
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    /*  update_from_pinpoint
    *   Update relative_robot_position using the most recent data from pinpoint
    * */
    public void update_from_pinpoint()
    {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double rot = pinpoint.getHeading(AngleUnit.DEGREES);
        this.relative_robot_position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, rot);
    }

    public void reset_pinpoint()
    {
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }

}
