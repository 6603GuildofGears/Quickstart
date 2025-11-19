package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp(name = "Assisted Aim", group = "Vision")
public class AssistedAim extends OpMode {

    // Hardware
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx intake, shooter;
    private Servo blocker;
    private Limelight3A limelight;

    // Constants
    private final double TICKS_PER_REV = 28;

    // Tunable Constants via FTC Dashboard
    public static double KP_ROTATE = 0.02;   // Proportional gain for auto-rotation
    public static double ROTATE_DEADBAND = 3.0;  // Degrees of error to ignore
    public static double MAX_AUTO_ROTATE_SPEED = 0.5;  // Max rotation speed during auto-aim

    // State
    private boolean autoAimEnabled = false;
    private boolean lastButtonState = false;

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

        // Drivetrain setup - Match Rango configuration
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake and Shooter setup
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight setup
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.setMsTransmissionInterval(11);
            limelight.pipelineSwitch(0);  // Use pipeline 0 for AprilTags
            limelight.start();
        } catch (Exception e) {
            telemetry.addLine("Warning: Limelight initialization failed");
            telemetry.addData("Error", e.getMessage());
        }

        telemetry.addLine("Assisted Aim Ready");
        telemetry.addLine("Press A to toggle auto-aim");
        telemetry.update();
    }

    @Override
    public void start() {
        blocker.setPosition(0.3);
    }

    @Override
    public void loop() {
        // Toggle auto-aim with A button
        boolean currentButtonState = gamepad1.a;
        if (currentButtonState && !lastButtonState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastButtonState = currentButtonState;

        // Handle subsystems
        handleSubsystemControls();

        // Handle drive with optional auto-aim
        if (autoAimEnabled) {
            assistedDrive();
        } else {
            manualDrive();
        }

        displayTelemetry();
    }

    /**
     * Manual drive - full control to driver
     */
    private void manualDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive(forward, strafe, rotate);
    }

    /**
     * Assisted drive - driver controls movement, auto-rotation to AprilTag
     */
    private void assistedDrive() {
        // Use EXACT same controls as Rango for forward/strafe - DO NOT CHANGE
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;  // Default to manual rotation

        // Auto-rotate to face AprilTag if visible and right stick is centered
        if (Math.abs(gamepad1.right_stick_x) < 0.1) {  // Only auto-aim when not manually rotating
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        LLResultTypes.FiducialResult tag = fiducials.get(0);
                        double tx = tag.getTargetXDegrees();

                        // Calculate rotation to face tag
                        if (Math.abs(tx) > ROTATE_DEADBAND) {
                            rotate = tx * KP_ROTATE;
                            rotate = clamp(rotate, -MAX_AUTO_ROTATE_SPEED, MAX_AUTO_ROTATE_SPEED);
                        } else {
                            rotate = 0;  // Aligned, no rotation needed
                        }
                    }
                }
            }
        }

        drive(forward, strafe, rotate);
    }

    /**
     * Mecanum drive logic
     */
    private void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize powers
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }

    /**
     * Handle subsystem controls (intake, shooter, blocker)
     */
    private void handleSubsystemControls() {
        // Intake control
        double rpm = 3500;
        if (gamepad2.right_bumper) {
            intake.setPower(1.0);
        } else if (gamepad2.right_trigger > 0.1) {
            intake.setVelocity(getTickSpeed(-rpm));
        } else {
            intake.setVelocity(0);
        }

        // Blocker servo control
        if (gamepad2.a) {
            blocker.setPosition(0.175);
        } else {
            blocker.setPosition(0.3);
        }

        // Shooter control
        if (gamepad2.left_bumper) {
            shooter.setPower(-0.8);
        } else if (gamepad2.left_trigger > 0.1) {
            shooter.setPower(0.6);
        } else {
            shooter.setPower(0);
        }
    }

    /**
     * Convert RPM to ticks per second
     */
    private double getTickSpeed(double speed) {
        return speed * TICKS_PER_REV / 60;
    }

    /**
     * Clamp value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Display telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("=== Assisted Aim ===");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addLine("Press A to toggle");
        telemetry.addLine();

        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);
                    telemetry.addLine("=== AprilTag Detected ===");
                    telemetry.addData("Tag ID", tag.getFiducialId());
                    telemetry.addData("X Offset (tx)", "%.2f deg", tag.getTargetXDegrees());
                    telemetry.addData("Area", "%.2f%%", tag.getTargetArea());

                    boolean aligned = Math.abs(tag.getTargetXDegrees()) < ROTATE_DEADBAND;
                    telemetry.addData("Aligned", aligned ? "✓" : "✗");
                } else {
                    telemetry.addLine("No AprilTag detected");
                }
            } else {
                telemetry.addLine("Waiting for Limelight data...");
            }
        } else {
            telemetry.addLine("Limelight not available");
        }

        telemetry.addLine();
        telemetry.addLine("=== Tuning ===");
        telemetry.addData("KP_ROTATE", KP_ROTATE);
        telemetry.addData("ROTATE_DEADBAND", ROTATE_DEADBAND);
        telemetry.addData("MAX_AUTO_ROTATE_SPEED", MAX_AUTO_ROTATE_SPEED);

        telemetry.update();
    }
}
