package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class PrattAuto extends OpMode {

    private Follower follower;
    private Timer timer;
    private Timer pathTimer;
    private Timer opmodeTimer;

    public enum PathState {
        FIRST_PATH,
        SHOOT_PRELOAD
    }

    PathState pathState;

    public final Pose startPose = new Pose(122, 125, Math.toRadians(217));  // Start facing away from field
    public final Pose shootPose = new Pose(93.25, 98, Math.toRadians(220));  // Shooting position
    public final Pose sample1 = new Pose(130, 84, Math.toRadians(0));  // Sample 1 - front faces sample
    public final Pose sample2 = new Pose(130, 60, Math.toRadians(0));  // Sample 2 - front faces sample
    public final Pose sample3 = new Pose(130, 37, Math.toRadians(0)); 


    private PathChain firstPath;

    public void buildPaths() {
        // Path 1: Start -> Shoot position
        firstPath = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
            .build();
    }

    public void pathUpdate() {
        switch (pathState) {
            case FIRST_PATH:
                follower.followPath(firstPath, true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    //Add shooter logic
                    telemetry.addLine("Path 1 Done");
                }

                // Shoot preload
                break;
        }
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        timer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        pathState = PathState.FIRST_PATH;

        buildPaths();

        follower.setPose(startPose);
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        double time = opmodeTimer.getElapsedTimeSeconds();
        telemetry.addData("Time (s): ", time);
        telemetry.update();


        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", "%.1f\"", follower.getPose().getX());    
        telemetry.addData("Y", "%.1f\"", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
    }
}