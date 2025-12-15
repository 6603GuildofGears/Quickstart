package org.firstinspires.ftc.teamcode;

// Imports for Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

// Limelight SDK imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

@Config
@TeleOp(name = "Camera Distance", group = "Robot")
public class CameraDistance extends OpMode {

    // Hardware variables - Use DcMotorEx for all motors for consistency
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo blocker;
    private IMU imu;

    // Limelight Variables
    private Limelight3A limelight;

    // Constants
    private final double TICKS_PER_REV = 28;
    private final double gear = 1.0;  // Gear ratio for rotation speed
    
    // PID coefficients for shooter stability (reduced P for less overshoot)
    private static final double SHOOTER_P = 1.5;   // Proportional (lower = less aggressive)
    private static final double SHOOTER_I = 0.15;  // Integral (helps eliminate steady-state error)
    private static final double SHOOTER_D = 0.1;   // Derivative (dampens oscillation)
    private static final double SHOOTER_F = 12.5;  // Feedforward (velocity control)
    
    // Exponential smoothing constants (tunable via FTC Dashboard)
    public static double ALPHA_COORDINATES = 0.3;   // Smoothing for X, Y, Z (0.3 = balanced)
    public static double ALPHA_DISTANCE = 0.25;     // Smoothing for distance (0.25 = smoother)
    
    // Camera coordinates storage (raw values)
    private double cameraX = 0.0;
    private double cameraY = 0.0;
    private double cameraZ = 0.0;
    private double targetDistance = 0.0;
    private double targetAngleX = 0.0;
    private double targetAngleY = 0.0;
    private int detectedTagId = -1;
    private boolean targetVisible = false;
    
    // Smoothed coordinates for display
    private double smoothedX = 0.0;
    private double smoothedY = 0.0;
    private double smoothedZ = 0.0;
    private double smoothedDistance = 0.0;
    private boolean firstReading = true;

    @Override
    public void init() {
        // Set up FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware mapping
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        blocker = hardwareMap.get(Servo.class, "blocker");
        imu = hardwareMap.get(IMU.class, "imu");

        // Drivetrain setup
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
            
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        // Intake and Shooter setup
        // NOTE: 'intake' variable is actually the shooter motor due to hardware swap
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Configure PID coefficients to reduce overshoot and oscillation on shooter
        intake.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU setup
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        blocker.setPosition(0.3);
    }

    @Override
    public void loop() {
        // Update camera data from Limelight
        updateCameraData();

        // Handle all robot controls
        handleSubsystemControls();
        handleDriveControls();

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Update camera position data from Limelight
     */
    private void updateCameraData() {
        if (limelight == null) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                
                // Get tag ID
                detectedTagId = (int) tag.getFiducialId();
                targetVisible = true;
                
                // Get target angles
                targetAngleX = tag.getTargetXDegrees();
                targetAngleY = tag.getTargetYDegrees();
                
                // Get robot pose relative to target
                Pose3D robotPose = tag.getRobotPoseTargetSpace();
                
                // Extract X, Y, Z coordinates (in meters) - raw values
                double rawX = robotPose.getPosition().x;
                double rawY = robotPose.getPosition().y;
                double rawZ = robotPose.getPosition().z;
                
                // Apply exponential smoothing to coordinates
                if (firstReading) {
                    // Initialize smoothed values on first reading
                    smoothedX = rawX;
                    smoothedY = rawY;
                    smoothedZ = rawZ;
                    firstReading = false;
                } else {
                    // Exponential smoothing: smoothed = (alpha * raw) + ((1 - alpha) * previous)
                    smoothedX = (ALPHA_COORDINATES * rawX) + ((1.0 - ALPHA_COORDINATES) * smoothedX);
                    smoothedY = (ALPHA_COORDINATES * rawY) + ((1.0 - ALPHA_COORDINATES) * smoothedY);
                    smoothedZ = (ALPHA_COORDINATES * rawZ) + ((1.0 - ALPHA_COORDINATES) * smoothedZ);
                }
                
                // Store smoothed values for display
                cameraX = smoothedX;
                cameraY = smoothedY;
                cameraZ = smoothedZ;
                
                // Calculate distance to target (3D distance) from raw values
                double rawDistance = Math.sqrt(
                    rawX * rawX + 
                    rawY * rawY + 
                    rawZ * rawZ
                );
                
                // Apply smoothing to distance
                if (firstReading) {
                    smoothedDistance = rawDistance;
                } else {
                    smoothedDistance = (ALPHA_DISTANCE * rawDistance) + ((1.0 - ALPHA_DISTANCE) * smoothedDistance);
                }
                
                targetDistance = smoothedDistance;
            } else {
                targetVisible = false;
                detectedTagId = -1;
                firstReading = true;  // Reset smoothing when target is lost
            }
        } else {
            targetVisible = false;
            detectedTagId = -1;
            firstReading = true;  // Reset smoothing when target is lost
        }
    }

    /**
     * Handles the logic for the intake, shooter, and blocker mechanisms using gamepad 2.
     */
    private void handleSubsystemControls() {
        // Intake motor control (simplified and corrected)
        double rpm = 3000;
        
        // Right bumper = full speed (NOTE: 'intake' variable is actually the shooter motor due to hardware swap)
        if(gamepad2.right_bumper && gamepad2.b) {
            intake.setVelocity(getTickSpeed(rpm + 500)); // far zone
        } else if (gamepad2.right_bumper) {
            intake.setVelocity(getTickSpeed(rpm)); // Full speed
        } else if (gamepad2.right_trigger > 0.1) { // Added a deadzone for the trigger
            intake.setVelocity(getTickSpeed(-rpm));
        } else {
            intake.setVelocity(0);
        }

        // Blocker servo control
        if (gamepad2.a) {
            blocker.setPosition(0.175);
        } else {
            blocker.setPosition(0.32);
        }

        // Shooter motor control - dpad_up for half speed (NOTE: 'shooter' variable is actually the intake motor due to hardware swap)
        if (gamepad2.left_bumper && gamepad2.a) { // Added a deadzone for the trigger
            shooter.setPower(-0.4);
        } else if (gamepad2.left_bumper) {
            shooter.setPower(-0.8);
        }  else if (gamepad2.left_trigger > 0.1) { // Added a deadzone for the trigger
            shooter.setPower(0.75);
        } else {
            shooter.setPower(0);
        }
    }

    /**
     * Handles the Mecanum drivetrain logic using gamepad 1.
     */
    private void handleDriveControls() {
        double LStickY = -gamepad1.left_stick_x;  // Reversed forward/back
        double LStickX = -gamepad1.left_stick_y;  // Reversed strafe
        double RStickX = gamepad1.right_stick_x;  // Reversed turn
        
        boolean LBumper1 = gamepad1.left_bumper;
        boolean RBumper1 = gamepad1.right_bumper;
        
        double gear = 1;

        if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
            double rotation = 0;

            double newX = -LStickX * Math.cos(rotation) - -LStickY * Math.sin(rotation);
            double newY = LStickY * Math.cos(rotation) - -LStickX * Math.sin(rotation);

            double r = Math.hypot(newX, newY);
            double robotAngle = Math.atan2(newY, newX) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;  // Match RStickX direction

            double v1 = r * Math.cos(robotAngle) + rightX * gear; //lf
            double v2 = r * Math.sin(robotAngle) + rightX * gear; //rf
            double v3 = r * Math.sin(robotAngle) - rightX * gear; //lb
            double v4 = r * Math.cos(robotAngle) - rightX * gear; //rb

            SetPower(v1, v2, v3, v4);

        } else if (LBumper1) {
            SetPower(-gear, gear, gear, -gear);

        } else if (RBumper1) {
            SetPower(gear, -gear, -gear, gear);

        } else {
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }

    /**
     * Converts RPM to ticks per second for the motor.
     */
    public double getTickSpeed(double speed) {
        return speed * TICKS_PER_REV / 60;
    }

    /**
     * Set power to all drive motors
     */
    private void SetPower(double v1, double v2, double v3, double v4) {
        frontLeftDrive.setPower(v1);
        frontRightDrive.setPower(v2);
        backLeftDrive.setPower(v3);
        backRightDrive.setPower(v4);
    }

    /**
     * Displays all telemetry data to the Driver Station and FTC Dashboard.
     */
    private void displayTelemetry() {
        telemetry.addLine("=== LIMELIGHT CAMERA DATA ===");
        telemetry.addLine();
        
        if (targetVisible) {
            telemetry.addData("Target Detected", "YES");
            telemetry.addData("AprilTag ID", detectedTagId);
            telemetry.addLine();
            
            telemetry.addLine("--- Camera Coordinates (meters) ---");
            telemetry.addData("X Position", "%.3f m", cameraX);
            telemetry.addData("Y Position", "%.3f m", cameraY);
            telemetry.addData("Z Position", "%.3f m", cameraZ);
            telemetry.addLine();
            
            telemetry.addLine("--- Camera Coordinates (inches) ---");
            telemetry.addData("X Position", "%.2f in", cameraX * 39.3701);
            telemetry.addData("Y Position", "%.2f in", cameraY * 39.3701);
            telemetry.addData("Z Position", "%.2f in", cameraZ * 39.3701);
            telemetry.addLine();
            
            telemetry.addLine("--- Target Information ---");
            telemetry.addData("Distance", "%.2f in (%.2f m)", targetDistance * 39.3701, targetDistance);
            telemetry.addData("Angle X", "%.2f°", targetAngleX);
            telemetry.addData("Angle Y", "%.2f°", targetAngleY);
        } else {
            telemetry.addData("Target Detected", "NO");
            telemetry.addData("Status", "No AprilTag visible");
        }
        
        telemetry.addLine();
        telemetry.addLine("--- Limelight Status ---");
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Latency", "%.1f ms", result.getCaptureLatency() + result.getTargetingLatency());
            telemetry.addData("Pipeline", limelight.getStatus().getPipelineIndex());
        } else {
            telemetry.addData("Limelight", "Waiting for data...");
        }

        telemetry.update();
    }
}
