package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Standalone, low-speed calibration of TWO independent flower servos plus
 * the original shooter-adjustment servo, using Panels Configurables.
 *
 * Both flower servos can be ENABLED at the same time, with different targets.
 * They are NOT automatically mirrored: their directions/angles depend on how
 * they are installed. Individual target values do not guarantee the connected
 * linkages won't bind. First test with mechanical linkages disconnected.
 *
 * Does NOT initialize motors, competition TeleOp, or Pedro Pathing.
 *
 * WARNING: Servo.getPosition() is a commanded position, NOT physical shaft
 * feedback. The first command can still move an uncalibrated servo from an
 * unknown physical starting position despite the rate limiter.
 */
@Configurable
@TeleOp(name = "TEST - Servo Calibration", group = "Testing")
public class ServoCalibration extends OpMode {

    // Live-adjustable in Panels > Configurables. Both flower servos can run
    // simultaneously. Start with small target changes, not full travel.
    // 0.50 is a UI starting value, NOT a verified safe physical angle.
    public static double flowerPosition = 0.50;
    public static double flower2Position = 0.50;
    public static double shooterPosition = 0.50;

    // Each servo has an independent enable switch. Both flower switches may
    // be true together. All are force-reset to false when the OpMode INITs.
    public static boolean flowerEnabled = false;
    public static boolean flower2Enabled = false;
    public static boolean shooterEnabled = false;

    // Fixed safety limit: normalized servo position units per second.
    private static final double MAX_POSITION_RATE = 0.10;
    private static final double MAX_LOOP_TIME_SECONDS = 0.10;
    private static final double COMMAND_EPSILON = 0.00001;

    private Servo flowerServo;
    private Servo flowerServo2;
    private Servo shooterServo;

    private boolean flowerObservedDisabled;
    private boolean flower2ObservedDisabled;
    private boolean shooterObservedDisabled;
    private boolean flowerArmed;
    private boolean flower2Armed;
    private boolean shooterArmed;
    private double flowerLastCommand = Double.NaN;
    private double flower2LastCommand = Double.NaN;
    private double shooterLastCommand = Double.NaN;
    private String flowerStatus = "NOT STARTED";
    private String flower2Status = "NOT STARTED";
    private String shooterStatus = "NOT STARTED";
    private long previousLoopNanos;

    @Override
    public void init() {
        // Panels static fields can survive across OpMode runs. Force disarm.
        flowerEnabled = false;
        flower2Enabled = false;
        shooterEnabled = false;

        flowerServo = hardwareMap.tryGet(Servo.class, "flowerServo");
        flowerServo2 = hardwareMap.tryGet(Servo.class, "flowerServo2");
        shooterServo = hardwareMap.tryGet(Servo.class, "shooterAdjust");

        flowerObservedDisabled = false;
        flower2ObservedDisabled = false;
        shooterObservedDisabled = false;
        flowerArmed = false;
        flower2Armed = false;
        shooterArmed = false;
        flowerLastCommand = Double.NaN;
        flower2LastCommand = Double.NaN;
        shooterLastCommand = Double.NaN;
        flowerStatus = flowerServo == null ? "NOT MAPPED" : "DISARMED";
        flower2Status = flowerServo2 == null ? "NOT MAPPED" : "DISARMED";
        shooterStatus = shooterServo == null ? "NOT MAPPED" : "DISARMED";

        // Do NOT call setPosition(), setDirection(), or any motor API in init.
        showTelemetry();
    }

    @Override
    public void start() {
        // Require each enabled channel to have been observed OFF after START.
        flowerObservedDisabled = false;
        flower2ObservedDisabled = false;
        shooterObservedDisabled = false;
        flowerArmed = false;
        flower2Armed = false;
        shooterArmed = false;
        previousLoopNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double elapsed = (now - previousLoopNanos) / 1_000_000_000.0;
        previousLoopNanos = now;
        // Prevent one long loop pause from allowing a large step.
        double dt = Double.isFinite(elapsed) && elapsed > 0.0
                ? Math.min(elapsed, MAX_LOOP_TIME_SECONDS) : 0.0;

        if (!flowerEnabled) {
            flowerObservedDisabled = true;
            flowerArmed = false;
            flowerStatus = flowerServo == null ? "NOT MAPPED" : "DISARMED (holds last position)";
        } else if (flowerServo == null) {
            flowerArmed = false;
            flowerStatus = "NOT MAPPED";
        } else if (!flowerObservedDisabled) {
            flowerArmed = false;
            flowerStatus = "Toggle OFF then ON after START";
        } else if (!isValidPosition(flowerPosition)) {
            flowerArmed = false;
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
                RobotLog.ee("ServoCalibration", e, "Flower servo 1 operation failed");
                flowerEnabled = false;
                flowerArmed = false;
                flowerObservedDisabled = false;
                flowerStatus = "HARDWARE ERROR; disarmed";
            }
        }

        if (!flower2Enabled) {
            flower2ObservedDisabled = true;
            flower2Armed = false;
            flower2Status = flowerServo2 == null ? "NOT MAPPED" : "DISARMED (holds last position)";
        } else if (flowerServo2 == null) {
            flower2Armed = false;
            flower2Status = "NOT MAPPED";
        } else if (!flower2ObservedDisabled) {
            flower2Armed = false;
            flower2Status = "Toggle OFF then ON after START";
        } else if (!isValidPosition(flower2Position)) {
            flower2Armed = false;
            flower2Status = "INVALID TARGET; no command";
        } else {
            flower2Armed = true;
            try {
                if (Double.isNaN(flower2LastCommand)) {
                    flower2LastCommand = validBaseline(flowerServo2.getPosition());
                }
                double next = approach(flower2LastCommand, flower2Position, MAX_POSITION_RATE * dt);
                if (Math.abs(next - flower2LastCommand) > COMMAND_EPSILON) {
                    flowerServo2.setPosition(next);
                    flower2LastCommand = next;
                }
                flower2Status = "ENABLED";
            } catch (RuntimeException e) {
                RobotLog.ee("ServoCalibration", e, "Flower servo 2 operation failed");
                flower2Enabled = false;
                flower2Armed = false;
                flower2ObservedDisabled = false;
                flower2Status = "HARDWARE ERROR; disarmed";
            }
        }

        if (!shooterEnabled) {
            shooterObservedDisabled = true;
            shooterArmed = false;
            shooterStatus = shooterServo == null ? "NOT MAPPED" : "DISARMED (holds last position)";
        } else if (shooterServo == null) {
            shooterArmed = false;
            shooterStatus = "NOT MAPPED";
        } else if (!shooterObservedDisabled) {
            shooterArmed = false;
            shooterStatus = "Toggle OFF then ON after START";
        } else if (!isValidPosition(shooterPosition)) {
            shooterArmed = false;
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
        flower2Enabled = false;
        shooterEnabled = false;
        flowerArmed = false;
        flower2Armed = false;
        shooterArmed = false;
        // STOP sends no position commands. PWM may still hold last targets.
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
        telemetry.addLine("TEST ONLY: verify both flower servos' directions before linking");
        telemetry.addLine("Panels: set BOTH flower targets, then enable each OFF -> ON after START");
        telemetry.addLine("WARNING: different servo positions can mechanically fight when linked");
        telemetry.addData("Flower 1 mapped", flowerServo != null);
        telemetry.addData("Flower 1 target", flowerPosition);
        telemetry.addData("Flower 1 last command", flowerLastCommand);
        telemetry.addData("Flower 1 armed", flowerArmed);
        telemetry.addData("Flower 1 status", flowerStatus);
        telemetry.addData("Flower 2 mapped", flowerServo2 != null);
        telemetry.addData("Flower 2 target", flower2Position);
        telemetry.addData("Flower 2 last command", flower2LastCommand);
        telemetry.addData("Flower 2 armed", flower2Armed);
        telemetry.addData("Flower 2 status", flower2Status);
        telemetry.addData("Shooter adjust mapped", shooterServo != null);
        telemetry.addData("Shooter target", shooterPosition);
        telemetry.addData("Shooter last command", shooterLastCommand);
        telemetry.addData("Shooter armed", shooterArmed);
        telemetry.addData("Shooter status", shooterStatus);
        telemetry.update();
    }
}
