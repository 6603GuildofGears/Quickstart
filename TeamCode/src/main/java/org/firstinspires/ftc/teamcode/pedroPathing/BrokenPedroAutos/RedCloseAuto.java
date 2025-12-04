package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red - Close MINIMAL", group = "Competition")
public class RedCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    
    // All drive motors declared to set directions in auto
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx backRightDrive;

    // Poses - INVERTED: front is now back (180° flip)
    // Robot back now faces samples, front faces basket
    public final Pose startPose = new Pose(122, 125, Math.toRadians(217));  // Start facing away from field
    public final Pose shootPose = new Pose(93.25, 98, Math.toRadians(220));  // Shooting position
    public final Pose sample1 = new Pose(130, 84, Math.toRadians(0));  // Sample 1 - front faces sample
    public final Pose sample2 = new Pose(130, 60, Math.toRadians(0));  // Sample 2 - front faces sample
    public final Pose sample3 = new Pose(130, 37, Math.toRadians(0));  // Sample 3 - front faces sample

    // Paths
    private Path path1, path2, path3, path4, path5, path6, path7;

    /**
     * Build all autonomous paths - SIMPLE
     */
    public void buildPaths() {
        // Path 1: Start -> Shoot position
        path1 = new Path(new BezierLine(startPose, shootPose));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // Path 2: Shoot -> Sample 1
        path2 = new Path(new BezierLine(shootPose, sample1));
        path2.setLinearHeadingInterpolation(shootPose.getHeading(), sample1.getHeading());

        // Path 3: Sample 1 -> Shoot
        path3 = new Path(new BezierLine(sample1, shootPose));
        path3.setLinearHeadingInterpolation(sample1.getHeading(), shootPose.getHeading());

        // Path 4: Shoot -> Sample 2
        path4 = new Path(new BezierLine(shootPose, sample2));
        path4.setLinearHeadingInterpolation(shootPose.getHeading(), sample2.getHeading());

        // Path 5: Sample 2 -> Shoot
        path5 = new Path(new BezierLine(sample2, shootPose));
        path5.setLinearHeadingInterpolation(sample2.getHeading(), shootPose.getHeading());

        // Path 6: Shoot -> Sample 3
        path6 = new Path(new BezierLine(shootPose, sample3));
        path6.setLinearHeadingInterpolation(shootPose.getHeading(), sample3.getHeading());

        // Path 7: Sample 3 -> Shoot
        path7 = new Path(new BezierLine(sample3, shootPose));
        path7.setLinearHeadingInterpolation(sample3.getHeading(), shootPose.getHeading());
    }

    /**
     * Main autonomous state machine - BARE MINIMUM
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(-1);  // Done
                }
                break;
        }
    }

    /**
     * Change path state and reset timer
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        
      
        
        buildPaths();
        
        follower.update();

        telemetry.addLine("Red - Close MINIMAL");
        telemetry.addLine("INVERTED: Front is now back");
        telemetry.addLine("---");
        telemetry.addData("Start Pose", "X=%.1f Y=%.1f H=%.0f°", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Pinpoint", "X=%.1f Y=%.1f H=%.0f°", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Paths Built", path1 != null ? "YES ✓" : "FAILED");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", "%.1f\"", follower.getPose().getX());
        telemetry.addData("Y", "%.1f\"", follower.getPose().getY());
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Time", "%.1fs", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
