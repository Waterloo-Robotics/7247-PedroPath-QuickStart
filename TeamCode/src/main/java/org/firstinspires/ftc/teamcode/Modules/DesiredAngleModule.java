package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DesiredAngleModule {
    /* Purpose of this file
     * The Eng goal of this file is to have the robot figure out
     * the angle it needs to turn to face the classifier
     *
     * GoalX-currentX=new x, goalY-currentY=new y
     * desired angle=tan-1 new y/new x
     * */


    public boolean on_red_side;
    double DesiredAngle;
    public DesiredAngleModule(boolean on_red_side)
    {
        this.on_red_side = on_red_side;

    }

    public double estimate_desired_angle(Pose2D robot_position) {

        if (on_red_side) {
            double x = robot_position.getX(DistanceUnit.INCH) + 58.3727;
            double y = robot_position.getY(DistanceUnit.INCH) - 55.6425;
            DesiredAngle = Math.atan2(y, x);
        }

        if(!on_red_side){
            double x = robot_position.getX(DistanceUnit.INCH) + 58.3727;
            double y = robot_position.getY(DistanceUnit.INCH) + 55.6425;
            DesiredAngle = Math.atan2(y, x);
        }
        /* Return value is 0 - 360 with 0 pointing towards the audience and counter clockwise is positive */
        return (Math.toDegrees(DesiredAngle) + 180) % 360;
    }
}
