package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class IndexerModule {
    public IndexerSpot artifact_1;
    public IndexerSpot artifact_2;
    public IndexerSpot artifact_3;

    private double GREENLED = 0.500;
    private double REDLED = 0.29;
    private double YELLOWLED = 0.388;
    private double LEDOFF = 0.0;
    public List<IndexerSpot> artifacts = new ArrayList<>();
    public int num_artifacts = 0;
    public Servo light1;

    public enum ModuleStates {
        WAITING,
        SHOOT_DELAY,
        SHOOT_INDIVIDUAL,
        SHOOT_ALL;
    }
    public ModuleStates module_state = ModuleStates.WAITING;

    private IndexerSpot last_fired_indexer;
    private ElapsedTime delay = new ElapsedTime();

    public IndexerModule(Servo a1_servo, RevColorSensorV3 a1_color_a, RevColorSensorV3 a1_color_b,
                         Servo a2_servo, RevColorSensorV3 a2_color_a, RevColorSensorV3 a2_color_b,
                         Servo a3_servo, RevColorSensorV3 a3_color_a, RevColorSensorV3 a3_color_b,
                         Servo light1)
    {
        this.artifact_1 = new IndexerSpot(a1_servo, a1_color_a, a1_color_b, 1, 0, 0.76);
        this.artifact_2 = new IndexerSpot(a2_servo, a2_color_a, a2_color_b, 1, 0, 0.73);
        this.artifact_3 = new IndexerSpot(a3_servo, a3_color_a, a3_color_b, 0, 1, 0.14);

        this.artifacts.add(this.artifact_1);
        this.artifacts.add(this.artifact_2);
        this.artifacts.add(this.artifact_3);
        this.light1 = light1;
    }

    public void update()
    {
        this.artifact_1.update();
        this.artifact_2.update();
        this.artifact_3.update();

        if (num_artifacts == 0) {
            light1.setPosition(LEDOFF);
        }
        else if (num_artifacts == 1) {
            light1.setPosition(GREENLED);
        }
        else if (num_artifacts == 2) {
            light1.setPosition(YELLOWLED);
        }
        else if (num_artifacts == 3) {
            light1.setPosition(REDLED);
        }
        /* Recount the number of artifacts currently in the indexer */
        this.num_artifacts = 0;
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status != ArtifactStatus.EMPTY)
            {
                this.num_artifacts++;
            }
        }

        switch (this.module_state)
        {
            case SHOOT_INDIVIDUAL:
                /* If for whatever reason we no longer have a last fired indexer, or the last
                /* fired indexer is just returned to home, return to the waiting state. */
                if (this.last_fired_indexer == null || this.last_fired_indexer.isHome())
                {
                    this.module_state = ModuleStates.WAITING;
                }

                break;
            case SHOOT_ALL:
                /* If there is a spot we just shot and it has returned back home, shoot the next spot */
                if (this.last_fired_indexer == null || (this.last_fired_indexer != null && this.last_fired_indexer.isHome()))
                {
                    boolean shot_next = this.shootNext();

                    /* If we ran out of artifacts to shoot, return to waiting state */
                    if (!shot_next)
                    {
                        this.module_state = ModuleStates.WAITING;
                    }
                }

                break;
            case SHOOT_DELAY:
                if (delay.seconds() > 0.1)
                {
                    this.last_fired_indexer.shoot();
                    this.module_state = ModuleStates.SHOOT_INDIVIDUAL;
                }

        }
    }

    public void shootGreen()
    {
        /* Loop through our artifacts, if one of them is green shoot it and exit this method. */
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status == ArtifactStatus.GREEN)
            {

                if (this.num_artifacts == 2)
                {
                    raiseEmptySpot();
                    this.module_state = ModuleStates.SHOOT_DELAY;
                    delay.reset();
                } else {
                    spot.shoot();
                }
                this.last_fired_indexer = spot;
                return;
            }
        }

        /* We looped through our artifacts and we didn't find a green...
        /* Loop through them again and just shoot the first one that isn't empty */
        this.shootNext();
    }

    public void shootPurple()
    {
        /* Loop through our artifacts, if one of them is purple shoot it and exit this method. */
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status == ArtifactStatus.PURPLE)
            {
                if (this.num_artifacts == 2)
                {
                    raiseEmptySpot();
                    this.module_state = ModuleStates.SHOOT_DELAY;
                    delay.reset();
                } else {
                    spot.shoot();
                }

                this.last_fired_indexer = spot;
                return;
            }
        }

        /* We looped through our artifacts and we didn't find a green...
        /* Loop through them again and just shoot the first one that isn't empty */
        this.shootNext();
    }

    public boolean shootNext()
    {
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status != ArtifactStatus.EMPTY)
            {
                if (this.num_artifacts == 2)
                {
                    raiseEmptySpot();
                    this.module_state = ModuleStates.SHOOT_DELAY;
                    delay.reset();
                } else {
                    spot.shoot();
                }

                this.last_fired_indexer = spot;
                /* True indicates that an artifact was found to shoot */
                return true;
            }
        }

        /* False indicates that there are no more artifacts to shoot */
        this.last_fired_indexer = null;
        return false;
    }

    public void shootAll()
    {
        this.module_state = ModuleStates.SHOOT_ALL;
    }

    private void raiseEmptySpot()
    {
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status == ArtifactStatus.EMPTY)
            {
                spot.blockMiddle();
            }
        }

    }
}
