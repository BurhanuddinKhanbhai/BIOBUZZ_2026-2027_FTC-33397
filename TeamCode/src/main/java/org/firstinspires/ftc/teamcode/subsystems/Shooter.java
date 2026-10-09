package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;

/** Controls two flywheel motors using SDK velocity control and an optional adjustment servo. */
public class Shooter {
    public enum BallType { UNKNOWN, POLLEN, NECTAR }
    public enum SelectionMode {
        AUTO,
        /** Legacy manual hold mode, retained for existing callers. */
        MANUAL,
        MANUAL_POLLEN, MANUAL_NECTAR
    }

    private final DcMotorEx flywheelA;
    private final DcMotorEx flywheelB;
    private double targetVelocity;
    private double velocityTolerance;
    private double stableTimeSeconds;
    private boolean trackingStableTime;
    private long withinToleranceSinceNanos;

    private Servo adjustmentServo;
    private boolean adjustmentPositionsConfigured;
    private double pollenPosition;
    private double nectarPosition;
    private double adjustmentTargetPosition = Double.NaN;
    private double adjustmentSettlingSeconds;
    private long adjustmentCommandNanos;
    private BallType selectedBallType = BallType.UNKNOWN;
    private SelectionMode selectionMode = SelectionMode.MANUAL;

    /** Uses FORWARD for both motors; readiness thresholds must be supplied explicitly. */
    public Shooter(HardwareMap hardwareMap, double velocityTolerance, double stableTimeSeconds) {
        this(hardwareMap, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                velocityTolerance, stableTimeSeconds);
    }

    /**
     * Configure each direction so intended shooting rotation reports positive velocity.
     * Map flywheelA to Control Hub motor port 3 and flywheelB to Expansion Hub motor port 3,
     * with each motor's encoder connected to its corresponding encoder port 3.
     *
     * @param velocityTolerance allowed error per motor in encoder ticks per second
     * @param stableTimeSeconds required observed time in tolerance; zero permits immediate readiness
     */
    public Shooter(HardwareMap hardwareMap, DcMotorSimple.Direction directionA,
                   DcMotorSimple.Direction directionB, double velocityTolerance,
                   double stableTimeSeconds) {
        if (hardwareMap == null || directionA == null || directionB == null) {
            throw new IllegalArgumentException("Hardware map and flywheel directions must not be null");
        }
        validateNonnegative(velocityTolerance, "Velocity tolerance");
        validateNonnegative(stableTimeSeconds, "Stable time");
        this.velocityTolerance = velocityTolerance;
        this.stableTimeSeconds = stableTimeSeconds;
        flywheelA = initializeMotor(hardwareMap, "flywheelA", directionA);
        flywheelB = initializeMotor(hardwareMap, "flywheelB", directionB);
    }

    /**
     * Adds the optional "shooterAdjust" servo on Control Hub servo port 0.
     * Existing constructors remain flywheel-only and never look up this servo.
     * Sets direction without commanding a position; presets require explicit configuration.
     * The settling interval is a time estimate in seconds, not position feedback.
     */
    public Shooter(HardwareMap hardwareMap, DcMotorSimple.Direction directionA,
                   DcMotorSimple.Direction directionB, double velocityTolerance,
                   double stableTimeSeconds, Servo.Direction adjustmentDirection,
                   double adjustmentSettlingSeconds) {
        this(hardwareMap, directionA, directionB, velocityTolerance, stableTimeSeconds);
        if (adjustmentDirection == null) {
            throw new IllegalArgumentException("Adjustment servo direction must not be null");
        }
        validateNonnegative(adjustmentSettlingSeconds, "Adjustment settling time");
        adjustmentServo = hardwareMap.get(Servo.class, "shooterAdjust");
        adjustmentServo.setDirection(adjustmentDirection);
        this.adjustmentSettlingSeconds = adjustmentSettlingSeconds;
    }

    /**
     * Commands both motors in encoder ticks per second. Zero stops and clears readiness.
     * Repeating the same target preserves the stability timer.
     * Invalid targets stop both motors and throw IllegalArgumentException.
     */
    public void setTargetVelocity(double ticksPerSecond) {
        if (!isFinite(ticksPerSecond) || ticksPerSecond < 0.0) {
            stop();
            throw new IllegalArgumentException("Target velocity must be finite and nonnegative");
        }
        if (ticksPerSecond == 0.0) {
            stop();
            return;
        }
        if (ticksPerSecond != targetVelocity) {
            trackingStableTime = false;
        }
        flywheelA.setVelocity(ticksPerSecond);
        flywheelB.setVelocity(ticksPerSecond);
        targetVelocity = ticksPerSecond;
    }

    /** Commands zero velocity without resetting encoders; the flywheels may still be slowing. */
    public void stop() {
        trackingStableTime = false;
        targetVelocity = 0.0;
        flywheelA.setVelocity(0.0);
        flywheelB.setVelocity(0.0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Returns signed measured encoder velocity in ticks per second. */
    public double getVelocityA() {
        return flywheelA.getVelocity();
    }

    /** Returns signed measured encoder velocity in ticks per second. */
    public double getVelocityB() {
        return flywheelB.getVelocity();
    }

    /** Target minus measured velocity in ticks per second; positive means below target. */
    public double getVelocityErrorA() {
        return targetVelocity - getVelocityA();
    }

    /** Target minus measured velocity in ticks per second; positive means below target. */
    public double getVelocityErrorB() {
        return targetVelocity - getVelocityB();
    }

    /**
     * Poll every control loop to track consecutive in-tolerance observations without blocking.
     * Both signed velocities must remain within tolerance for the configured time.
     * Zero target, invalid feedback, or an out-of-tolerance sample clears readiness.
     * Movement between polls cannot be observed.
     */
    public boolean isAtSpeed() {
        if (targetVelocity == 0.0) {
            trackingStableTime = false;
            return false;
        }
        double velocityA = getVelocityA();
        double velocityB = getVelocityB();
        if (!isFinite(velocityA) || !isFinite(velocityB)
                || Math.abs(targetVelocity - velocityA) > velocityTolerance
                || Math.abs(targetVelocity - velocityB) > velocityTolerance) {
            trackingStableTime = false;
            return false;
        }
        long now = System.nanoTime();
        if (!trackingStableTime) {
            withinToleranceSinceNanos = now;
            trackingStableTime = true;
        }
        return (now - withinToleranceSinceNanos) / 1_000_000_000.0 >= stableTimeSeconds;
    }

    /** Sets the finite, nonnegative tolerance in ticks per second; changes restart timing. */
    public void setVelocityTolerance(double velocityTolerance) {
        validateNonnegative(velocityTolerance, "Velocity tolerance");
        if (this.velocityTolerance != velocityTolerance) {
            trackingStableTime = false;
        }
        this.velocityTolerance = velocityTolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }

    /** Sets the finite, nonnegative stability duration in seconds; changes restart timing. */
    public void setStableTimeSeconds(double stableTimeSeconds) {
        validateNonnegative(stableTimeSeconds, "Stable time");
        if (this.stableTimeSeconds != stableTimeSeconds) {
            trackingStableTime = false;
        }
        this.stableTimeSeconds = stableTimeSeconds;
    }

    public double getStableTimeSeconds() {
        return stableTimeSeconds;
    }

    /**
     * Configures presets in the servo's configured direction without commanding movement.
     * Both values must be finite and within [0.0, 1.0]. Changes leave command history intact;
     * call a mode method explicitly to apply a new preset.
     */
    public void configureAdjustmentPositions(double pollen, double nectar) {
        validateAdjustmentPosition(pollen);
        validateAdjustmentPosition(nectar);
        pollenPosition = pollen;
        nectarPosition = nectar;
        adjustmentPositionsConfigured = true;
    }

    /** Manually selects pollen; requires the optional servo and configured presets. */
    public void setPollenMode() {
        commandAdjustment(BallType.POLLEN, pollenPosition);
        selectionMode = SelectionMode.MANUAL_POLLEN;
    }

    /** Manually selects nectar; requires the optional servo and configured presets. */
    public void setNectarMode() {
        commandAdjustment(BallType.NECTAR, nectarPosition);
        selectionMode = SelectionMode.MANUAL_NECTAR;
    }

    /** Enables AUTO without moving the servo; entering AUTO requires a fresh classification. */
    public void setAutoMode() {
        if (selectionMode != SelectionMode.AUTO) {
            selectedBallType = BallType.UNKNOWN;
        }
        selectionMode = SelectionMode.AUTO;
    }

    /**
     * Pass the sensor's filtered getBallType() result each loop, including UNKNOWN.
     * Manual overrides ignore sensor input. In AUTO, UNKNOWN/null clears selection validity
     * without moving the servo. Flywheel-only or uncalibrated operation also remains UNKNOWN.
     */
    public void update(BallColorSensor.BallType detectedBallType) {
        if (selectionMode != SelectionMode.AUTO) {
            return;
        }
        selectedBallType = BallType.UNKNOWN;
        if (adjustmentServo == null || !adjustmentPositionsConfigured || detectedBallType == null) {
            return;
        }
        switch (detectedBallType) {
            case POLLEN:
                commandAdjustment(BallType.POLLEN, pollenPosition);
                break;
            case NECTAR:
                commandAdjustment(BallType.NECTAR, nectarPosition);
                break;
            case UNKNOWN:
                break;
        }
    }

    /**
     * Poll each loop after update() in AUTO. Checks flywheel stability even when selection is
     * invalid. Requires a valid selection and the current calibrated preset to be commanded
     * and time-settled; it does not operate a feeder or confirm physical servo position.
     */
    public boolean isReadyToFeed() {
        boolean atSpeed = isAtSpeed();
        if (!atSpeed || selectedBallType == BallType.UNKNOWN
                || !adjustmentPositionsConfigured || !isAdjustmentSettled()) {
            return false;
        }
        double requiredPosition = selectedBallType == BallType.POLLEN ? pollenPosition : nectarPosition;
        return adjustmentTargetPosition == requiredPosition;
    }

    /** Returns the manual selection or latest valid AUTO selection; UNKNOWN is never feed-ready. */
    public BallType getSelectedBallType() {
        return selectedBallType;
    }

    /** Returns the last commanded position, or NaN before any command; never physical position. */
    public double getAdjustmentTargetPosition() {
        return adjustmentTargetPosition;
    }

    /**
     * Sets a finite, nonnegative settling interval. Changing it re-evaluates elapsed time
     * from the last position command; it does not move the servo or restart that timestamp.
     */
    public void setAdjustmentSettlingSeconds(double seconds) {
        validateNonnegative(seconds, "Adjustment settling time");
        adjustmentSettlingSeconds = seconds;
    }

    public double getAdjustmentSettlingSeconds() {
        return adjustmentSettlingSeconds;
    }

    /**
     * Reports elapsed-time settling only, not physical arrival. False without a position command.
     * Repeated requests for an unchanged position do not resend it or restart settling.
     */
    public boolean isAdjustmentSettled() {
        return adjustmentServo != null && !Double.isNaN(adjustmentTargetPosition)
                && (System.nanoTime() - adjustmentCommandNanos) / 1_000_000_000.0
                >= adjustmentSettlingSeconds;
    }

    /**
     * AUTO waits for update(); explicit manual modes command their configured preset.
     * Legacy MANUAL retains the current selection and servo target without movement.
     */
    public void setSelectionMode(SelectionMode mode) {
        if (mode == null) {
            throw new IllegalArgumentException("Shooter selection mode must not be null");
        }
        switch (mode) {
            case AUTO:
                setAutoMode();
                break;
            case MANUAL_POLLEN:
                setPollenMode();
                break;
            case MANUAL_NECTAR:
                setNectarMode();
                break;
            case MANUAL:
                selectionMode = SelectionMode.MANUAL;
                break;
        }
    }

    public SelectionMode getSelectionMode() {
        return selectionMode;
    }

    private void commandAdjustment(BallType ballType, double position) {
        if (adjustmentServo == null) {
            throw new IllegalStateException("Use the servo-aware Shooter constructor for adjustment commands");
        }
        if (!adjustmentPositionsConfigured) {
            throw new IllegalStateException("Configure adjustment positions before selecting a ball type");
        }
        if (Double.isNaN(adjustmentTargetPosition) || position != adjustmentTargetPosition) {
            adjustmentServo.setPosition(position);
            adjustmentTargetPosition = position;
            adjustmentCommandNanos = System.nanoTime();
        }
        selectedBallType = ballType;
    }

    private static void validateAdjustmentPosition(double position) {
        if (!isFinite(position) || position < 0.0 || position > 1.0) {
            throw new IllegalArgumentException("Adjustment position must be finite and between 0.0 and 1.0");
        }
    }

    private static DcMotorEx initializeMotor(HardwareMap hardwareMap, String name,
                                            DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(0.0);
        return motor;
    }

    private static void validateNonnegative(double value, String name) {
        if (!isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and nonnegative");
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
