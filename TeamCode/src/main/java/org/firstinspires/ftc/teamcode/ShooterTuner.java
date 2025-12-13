package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.bylazar.configurables.annotations.Configurable;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Configurable
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {
    
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo blocker;
    
    private boolean shooterRunning = false;
    private boolean lastAState = false;
    
    // Tunable via web panels at http://192.168.43.1:8001/
    public static double SHOOTER_P = 1.5;
    public static double SHOOTER_I = 0.15;
    public static double SHOOTER_D = 0.1;
    public static double SHOOTER_F = 12.5;
    
    // Target RPM - also tunable via panels
    public static double TARGET_RPM = 2500;
    
    // Constants
    private final double SHOOTER_TICKS_PER_REV = 1425.1;  // 5203 motor
    
    @Override
    public void init() {
        // Setup dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // NOTE: 'intakeMotor' config name is actually the shooter motor (hardware swap)
        shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        blocker = hardwareMap.get(Servo.class, "blocker");
        
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        blocker.setPosition(0.32);  // Closed position
        
        telemetry.addLine("=== Shooter PIDF Tuner ===");
        telemetry.addLine("Position robot at Red Close Auto start:");
        telemetry.addLine("X=122, Y=125, Heading=37°");
        telemetry.addLine("(at shoot pose for tuning)");
        telemetry.addLine();
        telemetry.addLine("Go to http://192.168.43.1:8001/");
        telemetry.addLine("Tune SHOOTER_P, I, D, F, TARGET_RPM");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A = Toggle shooter ON/OFF");
        telemetry.addLine("Right Bumper = Open blocker");
        telemetry.addLine("Left Bumper = Run intake (feed)");
        telemetry.addLine("DPad Up/Down = Adjust RPM ±100");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Update PIDF coefficients every loop (so panel changes apply immediately)
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        
        // Toggle shooter on/off with A button
        if (gamepad1.a && !lastAState) {
            shooterRunning = !shooterRunning;
        }
        lastAState = gamepad1.a;
        
        // Manual RPM adjustment with DPad
        if (gamepad1.dpad_up) {
            TARGET_RPM += 100;
            if (TARGET_RPM > 6000) TARGET_RPM = 6000;  // Max 6000 RPM
        }
        if (gamepad1.dpad_down) {
            TARGET_RPM -= 100;
            if (TARGET_RPM < 0) TARGET_RPM = 0;  // Min 0 RPM
        }
        
        // Run shooter if toggled on
        if (shooterRunning) {
            double targetVelocity = getTickSpeed(TARGET_RPM);
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setVelocity(0);
        }
        
        // Blocker control - Right bumper to open (like in Rango)
        if (gamepad1.right_bumper) {
            blocker.setPosition(0.175);  // Open blocker
        } else {
            blocker.setPosition(0.055);  // Close blocker
        }
        
        // Intake control - Left bumper to feed
        if (gamepad1.left_bumper) {
            intake.setPower(-1.0);  // Run intake to feed sample
        } else {
            intake.setPower(0);  // Stop intake
        }
        double currentTicksPerSec = shooter.getVelocity();
        double currentRPM = (currentTicksPerSec / SHOOTER_TICKS_PER_REV) * 60;
        double error = TARGET_RPM - Math.abs(currentRPM);
        
        // Display telemetry
        telemetry.addLine("=== SHOOTER TUNING ===");
        telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", Math.abs(currentRPM));
        telemetry.addData("Error", "%.0f RPM", error);
        telemetry.addLine();
        telemetry.addLine("=== PIDF VALUES ===");
        telemetry.addData("P (Proportional)", "%.2f", SHOOTER_P);
        telemetry.addData("I (Integral)", "%.3f", SHOOTER_I);
        telemetry.addData("D (Derivative)", "%.2f", SHOOTER_D);
        telemetry.addData("F (Feedforward)", "%.2f", SHOOTER_F);
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A Button", shooterRunning ? "ON (press to stop)" : "OFF (press to start)");
        telemetry.addData("Right Bumper", gamepad1.right_bumper ? "Blocker OPEN" : "Blocker CLOSED");
        telemetry.addData("Left Bumper", gamepad1.left_bumper ? "Feeding" : "Stopped");
        telemetry.addLine();
        telemetry.addLine("Tune at: http://192.168.43.1:8001/");
        telemetry.addLine("Position doesn't affect tuning much,");
        telemetry.addLine("but for consistency use auto shoot pose");
        telemetry.update();
    }
    
    /**
     * Convert RPM to encoder ticks per second
     */
    private double getTickSpeed(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60;
    }
}
