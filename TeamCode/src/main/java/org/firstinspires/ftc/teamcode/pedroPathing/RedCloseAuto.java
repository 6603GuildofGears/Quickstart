package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PEDRO - Red Close Auto", group = "Red")
public class RedCloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    
    // Shooter hardware
    private DcMotorEx shooter;  // Note: in hardware config this is labeled "intakeMotor"
    private DcMotorEx intake;   // Note: in hardware config this is labeled "shooterMotor"
    private Servo blocker;
    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;
    
    // Shooter constants
    private final double TICKS_PER_REV = 28;
    private static final double SHOOTER_P = 2.0;
    private static final double SHOOTER_I = 0.15;
    private static final double SHOOTER_D = 0.1;
    private static final double SHOOTER_F = 12.5;

    public enum PathState {
        // start pos to end pos
        //drive = movment 
        //shoot = shooting preloaded elments
         
      DRIVE_STARTPOSE_TO_SHOOTPOSE,
      SHOOT_PRELOAD,
      DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE,
     DRIVE_INTAKEPOSE_TO_SAMPLE1,
        DRIVE_SAMPLE1_TO_SHOOTPOSE,
        SHOOT_SAMPLE1,
        DRIVE_SHOOTPOSE_TO_INTAKEPOSE2,
        DRIVE_INTAKEPOSE2_TO_SAMPLE2,
        DRIVE_SAMPLE2_TO_SHOOTPOSE,
        SHOOT_SAMPLE2,
        DRIVE_SHOOTPOSE_TO_ENDPOSE,

    }



      PathState pathState;


    private final Pose startPose = new Pose(122, 125, Math.toRadians(37));  // Start facing away from field
   
    private final Pose shootPose = new Pose(87, 92, Math.toRadians(40));    // Shooting position

    private final Pose shootPoseAlpha = new Pose(87, 92, Math.toRadians(30));    // Shooting position


    private final Pose intakePose = new Pose(112, 95, Math.toRadians(180));    // Intake position


    private final Pose sample1 = new Pose(130, 95, Math.toRadians(180)); //114, 84

    private final Pose intakePose2 = new Pose(102, 72.5, Math.toRadians(180));


    private final Pose sample2 = new Pose(134, 72.5, Math.toRadians(180)); //114, 60

    private final Pose intakePose3 = new Pose(112, 50, Math.toRadians(180));

    private final Pose sample3 = new Pose(134, 50, Math.toRadians(180)); //114, 40

    private final Pose endPose = new Pose(91, 115, Math.toRadians(40));  // End position (same as shoot pose for now)

    
    

    double rpm = 2750;

    private PathChain driveStartPoseShootPose;
    private PathChain driveShootPreloadToIntakePose;
    private PathChain driveIntakePoseToSample1;
    private PathChain driveSample1ToShootPose;
    private PathChain driveShootPoseToIntakePose2;
    private PathChain driveIntakePose2ToSample2;
    private PathChain driveSample2ToShootPose;
    private PathChain driveShootPoseToEndPose;

    public void buildPaths() {
     
     driveStartPoseShootPose = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setConstantHeadingInterpolation(shootPose.getHeading())
            .build();

     driveShootPreloadToIntakePose = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, intakePose))
            .setConstantHeadingInterpolation(intakePose.getHeading())
            .build();
        driveIntakePoseToSample1 = follower.pathBuilder()
            .addPath(new BezierLine(intakePose, sample1))
            .setConstantHeadingInterpolation(sample1.getHeading())
            .build();
        driveSample1ToShootPose = follower.pathBuilder()
            .addPath(new BezierLine(sample1, shootPoseAlpha))
            .setLinearHeadingInterpolation(sample1.getHeading(), shootPoseAlpha.getHeading())
            .build();
        driveShootPoseToIntakePose2 = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, intakePose2))
            .setConstantHeadingInterpolation(intakePose2.getHeading())
            .build();   
        driveIntakePose2ToSample2 = follower.pathBuilder()
            .addPath(new BezierLine(intakePose2, sample2))
            .setConstantHeadingInterpolation(sample2.getHeading())
            .build();
        driveSample2ToShootPose = follower.pathBuilder()
            .addPath(new BezierLine(sample2, shootPoseAlpha))
            .setLinearHeadingInterpolation(sample2.getHeading(), shootPoseAlpha.getHeading())
            .build();
        driveShootPoseToEndPose = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, endPose))
            .setConstantHeadingInterpolation(endPose.getHeading())
            .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseShootPose, true);
                    pathStarted = true;
                }
                
                // Start shooter when path is halfway done
                if (follower.getCurrentTValue() >= 0.4 && !shooterStarted) {
                    shooter.setVelocity(getTickSpeed(rpm));
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_PRELOAD;
                    pathStarted = false;
                }
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    // Wait 2 seconds then open blocker and run intake
                    if (shooterTimer.seconds() >= 2.0) {
                        blocker.setPosition(0.15);  // Open blocker
                        intake.setPower(-0.6);  // Run intake motor
                    }
                    
                    // After 6 seconds total, move to next state
                    if (shooterTimer.seconds() >= 6) {
                        pathState = PathState.DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE;
                        shooterStarted = false;
                    }
                 
                    telemetry.addLine("Path 1 Done");
                }
                break;

            case DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPreloadToIntakePose, true);
                    
                    // Turn off shooter and close blocker when moving to intake
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    blocker.setPosition(0.32);  // Close blocker
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE_TO_SAMPLE1;
                    pathStarted = false;
                }
                break;


            case DRIVE_INTAKEPOSE_TO_SAMPLE1:
                if (!pathStarted) {
                    follower.followPath(driveIntakePoseToSample1, true);
                    
                    // Turn on intake during path
                    intake.setPower(-0.575);
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_SAMPLE1_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;
            case DRIVE_SAMPLE1_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveSample1ToShootPose, true);
                    pathStarted = true;
                }
                
                // Start shooter when path is halfway done
                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                    shooter.setVelocity(getTickSpeed(rpm));
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_SAMPLE1;
                    pathStarted = false;
                }
                break;
                
            case SHOOT_SAMPLE1:
                if (!follower.isBusy()) {
                    // Wait 2 seconds then open blocker and run intake
                    if (shooterTimer.seconds() >= 2.0) {
                        blocker.setPosition(0.15);  // Open blocker
                        intake.setPower(-0.475);  // Run intake motor
                    }
                    
                    // After 6 seconds total, move to next state
                    if (shooterTimer.seconds() >= 6) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKEPOSE2;
                        shooterStarted = false;
                    }
                    
                    telemetry.addLine("Sample 1 Shot");
                }
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKEPOSE2:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntakePose2, true);
                    // Turn off shooter and close blocker when moving to intake
                    shooter.setVelocity(0);  
                    intake.setPower(0);
                    blocker.setPosition(0.32);  // Close blocker
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE2_TO_SAMPLE2;
                    pathStarted = false;
                }
                break;  
            case DRIVE_INTAKEPOSE2_TO_SAMPLE2:
                if (!pathStarted) {
                    follower.followPath(driveIntakePose2ToSample2, true);
                    
                    // Turn on intake during path
                    intake.setPower(-0.575);
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_SAMPLE2_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;
            case DRIVE_SAMPLE2_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveSample2ToShootPose, true);
                    pathStarted = true;
                }
                
                // Start shooter when path is halfway done
                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                    shooter.setVelocity(getTickSpeed(rpm));
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_SAMPLE2;
                    pathStarted = false;
                }
                break;

            case SHOOT_SAMPLE2:
                if (!follower.isBusy()) {
                    // Wait 2 seconds then open blocker and run intake
                    if (shooterTimer.seconds() >= 2.0) {
                        blocker.setPosition(0.15);  // Open blocker
                        intake.setPower(-0.65);  // Run intake motor
                    }
                    
                    telemetry.addLine("Sample 2 Shot");
                    
                    // After 6 seconds total, move to end position
                    if (shooterTimer.seconds() >= 6) {
                        // Turn off shooter and intake, close blocker
                        shooter.setVelocity(0);
                        intake.setPower(0);
                        blocker.setPosition(0.32);  // Close blocker
                        
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        shooterStarted = false;
                    }
                }
                break;
        
                case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    telemetry.addLine("Auto Complete");
                    pathStarted = false;
                    requestOpModeStop();
                }
                break;


            default:
                telemetry.addLine("No valid path state");
                break;   
         
        }
    
    }

    public void statePathUpdate( PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
    
    pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    follower = Constants.createFollower(hardwareMap);
    
    // Initialize shooter hardware
    shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");  // Actual shooter motor
    intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");   // Actual intake motor
    blocker = hardwareMap.get(Servo.class, "blocker");
    
    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    blocker.setPosition(0.32);  // Closed position

    buildPaths();
    follower.setPose(startPose);

    }

    public void start () {
    opmodeTimer.resetTimer();
    pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;

    }


    @Override
    public void loop() {

        follower.update();
        
        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());       
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    
    }
    
    public double getTickSpeed(double speed) {
        return speed * TICKS_PER_REV / 60;
    }
}
