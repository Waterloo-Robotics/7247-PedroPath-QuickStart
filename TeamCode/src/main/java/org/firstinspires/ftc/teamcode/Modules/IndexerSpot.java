package org.firstinspires.ftc.teamcode.Modules;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerSpot {
    /* IndexerSpot
    /* This class represents a single spot in the indexer. */
    private Servo servo;
    private RevColorSensorV3 color_a;
    private RevColorSensorV3 color_b;

    public ArtifactStatus artifact_status = ArtifactStatus.EMPTY;

    private final double home_position;
    private final double shoot_position;
    private final double middle_position;

    private enum ModuleStates {
        HOME,
        SHOOTING,
        RETURNING_HOME;
    }

    private ModuleStates module_state = ModuleStates.HOME;
    private final ElapsedTime servo_timer = new ElapsedTime();

    public boolean return_to_middle;

    public IndexerSpot(Servo spot_servo,
                       RevColorSensorV3 spot_color_a,
                       RevColorSensorV3 spot_color_b,
                       double home,
                       double shoot,
                       double middle)
    {
        this.servo = spot_servo;
        this.color_a = spot_color_a;
        this.color_b = spot_color_b;
        this.home_position = home;
        this.shoot_position = shoot;
        this.middle_position = middle;
        this.return_to_middle = false;

        this.servo.setPosition(this.home_position);
    }

    public void updateArtifactStatus()
    {
        NormalizedRGBA results_a = this.color_a.getNormalizedColors();
        NormalizedRGBA results_b = this.color_a.getNormalizedColors();
        int red   = (int)(results_a.red   + results_b.red) * 255 / 2;
        int green = (int)(results_a.green + results_b.green) * 255 / 2;
        int blue  = (int)(results_a.blue  + results_b.blue) * 255 / 2;

        // Use distance to detect if a ball is present
        double avgDistance = (this.color_a.getDistance(DistanceUnit.CM) +
                this.color_b.getDistance(DistanceUnit.CM)) / 2.0;

        if (avgDistance > 3.0) {
            this.artifact_status = ArtifactStatus.EMPTY;
        }
        else if (green > red + 50 && green > blue + 50) {
            this.artifact_status = ArtifactStatus.GREEN;
        } else if (red + blue > green + 80) {
            this.artifact_status = ArtifactStatus.PURPLE;
        } else {
            this.artifact_status = ArtifactStatus.GREEN;
        }
    }

    public void shoot()
    {
        this.servo.setPosition(this.shoot_position);
        this.servo_timer.reset();

        this.module_state = ModuleStates.SHOOTING;
    }

    public void blockMiddle()
    {
        this.servo.setPosition(this.middle_position);
        this.servo_timer.reset();

        this.module_state = ModuleStates.SHOOTING;
    }

    public boolean isHome(){
        return (this.module_state == ModuleStates.HOME);
    }

    public void update()
    {
        this.updateArtifactStatus();

        switch (this.module_state)
        {
            case SHOOTING:
                /* If enough time has elapsed for the artifact to put into the
                /* turret, then reset the timer and lower the arm */
                if (this.servo_timer.seconds() > 0.4)
                {
                    this.servo_timer.reset();

                    if (this.return_to_middle)
                    {
                        this.servo.setPosition(this.middle_position);
                        this.return_to_middle = false;
                    }
                    else
                    {
                        this.servo.setPosition(this.home_position);
                    }
                    this.module_state = ModuleStates.RETURNING_HOME;
                }
                break;

            case RETURNING_HOME:
                /* If enough time has elapsed for the arm to return home,
                /* stop the timer and return status to home */
                if (this.servo_timer.seconds() > 0.4)
                {
                    this.servo_timer.reset();

                    this.module_state = ModuleStates.HOME;
                }
                break;
        }

    }
}