package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FCDrivebaseModule {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    Boolean on_red_side;

    GoBildaPinpointDriver pinpoint;
    public FCDrivebaseModule(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, GoBildaPinpointDriver pinpoint, Boolean on_red_side)
    {
        this.pinpoint = pinpoint;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.on_red_side = on_red_side;
    }

    public void update_Drive(double x, double y, double turn)
    {
        /* Since the coordinate system present in the Pinpoint is the fixed FTC coordinate system,
           the heading will be the exact same if we're on Red or Blue alliance. In order for field
           centric to work, the heading used for the drivebase needs to be 180 degrees
           offset from each other. */
        double botHeading;
        if (this.on_red_side)
        {
            botHeading = this.pinpoint.getHeading(AngleUnit.RADIANS) - (Math.PI/2);  //=0 ODO IMU
        }
        else
        {
            botHeading = this.pinpoint.getHeading(AngleUnit.RADIANS) + (Math.PI/2);  //=0 ODO IMU
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        this.frontLeft.setPower(frontLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backLeft.setPower(backLeftPower);
        this.backRight.setPower(backRightPower);
    }

}
