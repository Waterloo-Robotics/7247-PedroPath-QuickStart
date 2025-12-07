package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@Autonomous(name = "Auto", group = "Examples")
public class TestautoMeetTwo extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    int counter;
    private final Pose startPose = new Pose(120, 121, Math.toRadians(35)); // Start Pose of our robot.
    private final Pose score1 = new Pose(96, 96, Math.toRadians(45));
    private final Pose pickupstart = new Pose(96,53, Math.toRadians(0));
    private final Pose pickupend = new Pose(110,53, Math.toRadians(0));
    private final Pose score2 = new Pose(96, 96, Math.toRadians(45));
    private final Pose endPose = new Pose(85, 48, Math.toRadians(90));// park Pose of our robot.
    private Follower follower;
    private Path scorePreload;
    private PathChain pickupmotifstart, pickupmotifend, score2path, park;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, score1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1.getHeading());

        pickupmotifstart = follower.pathBuilder()
                .addPath(new BezierLine(score1, pickupstart))
                .setLinearHeadingInterpolation(score1.getHeading(), pickupstart.getHeading())
                .build();

        pickupmotifend = follower.pathBuilder()
                .addPath(new BezierLine(pickupstart, pickupend))
                .setLinearHeadingInterpolation(pickupstart.getHeading(), pickupend.getHeading())
                .build();

        score2path = follower.pathBuilder()
                .addPath(new BezierLine(pickupend, score2))
                .setLinearHeadingInterpolation(pickupend.getHeading(), score2.getHeading())
                .build();


        park = follower.pathBuilder()
                .addPath(new BezierLine(score2, endPose))
                .setLinearHeadingInterpolation(score2.getHeading(), endPose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park, true);
                    setPathState(2);
                }

                break;

            case 2:

                if (!follower.isBusy()) {
                    follower.followPath(pickupmotifstart, true);

                    setPathState(3);
                }

                break;
            case 3:
                counter = counter + 1;
                if(counter == 60) {
                    follower.followPath(pickupmotifend, true);
                    setPathState(4);
                }

                break;

            case 4:

                if (!follower.isBusy()) {
                    follower.followPath(score2path, true);
                    setPathState(5);
                }

                break;
            case 5:

                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(6);
                }

                break;


        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.Hardware robot = new Constants.Hardware();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);


    }

}