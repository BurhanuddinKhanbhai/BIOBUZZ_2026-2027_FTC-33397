package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Standalone, low-speed calibration of two positional servos using Panels.
 * Does NOT initialize motors, the competition TeleOp, or Pedro Pathing.
 *
 * WARNING: Servo.getPosition() is a commanded position, NOT a measurement of
 * the physical servo shaft. Disconnect linkages for the first test: even the
 * first small command can move a shaft from an unknown physical position.
 */
@Configurable
@TeleOp(name = "TEST - Servo Calibration", group = "Testing")
public class ServoCalibration extends OpMode {

    // Live-adjustable in Panels > Configurables. Set one position at a time.
    // 0.50 is only the initial UI target, NOT a verified safe mechanism angle.
    public static double flowerPosition = 0.50;
    public static double shooterPosition = 0.50;
    public static boolean flowerEnabled = false;
    public static boolean shooterEnabled = false;

    // Fixed safety limit: normalized servo position units per second.
    private static final double MAX_POSITION_RATE = 0.10;
    private static final double MAX_LOOP_TIME_SECONDS = 0.10;
    private static final double COMMAND_EPSILON = 0.00001;

    private Servo flowerServo;
    private Servo shooterServo;

    private boolean flowerObservedDisabled;
    private boolean shooterObservedDisabled;
    private boolean flowerArmed;
    private boolean shooterArmed;
    private double flowerLastCommand = Double.NaN;
    private double shooterLastCommand = Double.NaN;
    private String flowerStatus = "NOT STARTED";
    private String shooterStatus = "NOT STARTED";
    private long previousLoopNanos;

    @Override
    public void init() {
        // Static Panels fields can survive across OpMode runs; always disarm.
        flowerEnabled = false;
        shooterEnabled = false;
        flowerServo = hardwareMap.tryGet(Servo.class, "flowerServo");
        shooterServo = hardwareMap.tryGet(Servo.class, "shooterAdjust");

        flowerObservedDisabled = false;
        shooterObservedDisabled = false;
        flowerArmed = false;
        shooterArmed = false;
        flowerLastCommand = Double.NaN;
        shooterLastCommand = Double.NaN;
        flowerStatus = flowerServo == null ? "NOT MAPPED" : "DISARMED";
        shooterStatus = shooterServo == null ? "NOT MAPPED" : "DISARMED";
        // Do NOT call setPosition(), setDirection(), or any motor API in init.
        showTelemetry();
    }

    @Override
    public void start() {
        // Require seeing DISABLED after START, followed by ENABLED, before motion.
        // This protects against stale Panels state from a previous run.
        flowerObservedDisabled = false;
        shooterObservedDisabled = false;
        flowerArmed = false;
        shooterArmed = false;
        previousLoopNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double elapsed = (now - previousLoopNanos) / 1_000_000_000.0;
        previousLoopNanos = now;
        // A long pause must not cause a big jump in a single control loop.
        double dt = Double.isFinite(elapsed) && elapsed > 0.0
                ? Math.min(elapsed, MAX_LOOP_TIME_SECONDS) : 0.0;

        if (!flowerEnabled) {
            flowerObservedDisabled = true;
            flowerArmed = false;
            flowerStatus = flowerServo == null ? "NOT MAPPED" : "DISARMED (holds last position)";
        } else if (flowerServo == null) {
            flowerStatus = "NOT MAPPED";
        } else if (!flowerObservedDisabled) {
            flowerStatus = "Toggle OFF then ON after START";
        } else if (!isValidPosition(flowerPosition)) {
            flowerStatus = "INVALID TARGET; no command";
        } else {
            flowerArmed = true;
            try {
                if (Double.isNaN(flowerLastCommand)) {
                    flowerLastCommand = validBaseline(flowerServo.getPosition());
                }
                double next = approach(flowerLastCommand, flowerPosition, MAX_POSITION_RATE * dt);
                if (Math.abs(next - flowerLastCommand) > COMMAND_EPSILON) {
                    flowerServo.setPosition(next);
                    flowerLastCommand = next;
                }
                flowerStatus = "ENABLED";
            } catch (RuntimeException e) {
                RobotLog.ee("ServoCalibration", e, "Flower servo operation failed");
                flowerEnabled = false;
                flowerArmed = false;
                flowerObservedDisabled = false;
                flowerStatus = "HARDWARE ERROR; disarmed";
            }
        }

        if (!shooterEnabled) {
            shooterObservedDisabled = true;
            shooterArmed = false;
            shooterStatus = shooterServo == null ? "NOT MAPPED" : "DISARMED (holds last position)";
        } else if (shooterServo == null) {
            shooterStatus = "NOT MAPPED";
        } else if (!shooterObservedDisabled) {
            shooterStatus = "Toggle OFF then ON after START";
        } else if (!isValidPosition(shooterPosition)) {
            shooterStatus = "INVALID TARGET; no command";
        } else {
            shooterArmed = true;
            try {
                if (Double.isNaN(shooterLastCommand)) {
                    shooterLastCommand = validBaseline(shooterServo.getPosition());
                }
                double next = approach(shooterLastCommand, shooterPosition, MAX_POSITION_RATE * dt);
                if (Math.abs(next - shooterLastCommand) > COMMAND_EPSILON) {
                    shooterServo.setPosition(next);
                    shooterLastCommand = next;
                }
                shooterStatus = "ENABLED";
            } catch (RuntimeException e) {
                RobotLog.ee("ServoCalibration", e, "Shooter servo operation failed");
                shooterEnabled = false;
                shooterArmed = false;
                shooterObservedDisabled = false;
                shooterStatus = "HARDWARE ERROR; disarmed";
            }
        }

        showTelemetry();
    }

    @Override
    public void stop() {
        flowerEnabled = false;
        shooterEnabled = false;
        flowerArmed = false;
        shooterArmed = false;
        // No position commands on STOP; positional servos may continue holding
        // their last target until PWM is disabled or the robot is powered off.
    }

    private static boolean isValidPosition(double position) {
        return Double.isFinite(position) && position >= 0.0 && position <= 1.0;
    }

    private static double validBaseline(double position) {
        return isValidPosition(position) ? position : 0.50;
    }

    private static double approach(double current, double target, double maxChange) {
        return Math.max(current - maxChange, Math.min(current + maxChange, target));
    }

    private void showTelemetry() {
        telemetry.addLine("TEST ONLY: Linkages disconnected for initial servo calibration");
        telemetry.addLine("Panels: toggle enable OFF -> ON after OpMode START");
        telemetry.addData("Flower mapped", flowerServo != null);
        telemetry.addData("Flower target", flowerPosition);
        telemetry.addData("Flower last command", flowerLastCommand);
        telemetry.addData("Flower armed", flowerArmed);
        telemetry.addData("Flower status", flowerStatus);
        telemetry.addData("Shooter adjust mapped", shooterServo != null);
        telemetry.addData("Shooter target", shooterPosition);
        telemetry.addData("Shooter last command", shooterLastCommand);
        telemetry.addData("Shooter armed", shooterArmed);
        telemetry.addData("Shooter status", shooterStatus);
        telemetry.update();
    }
}
