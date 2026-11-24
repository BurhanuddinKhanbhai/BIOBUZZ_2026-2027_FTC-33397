package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

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
    private double pollenPosition = Double.NaN;
    private double nectarPosition = Double.NaN;
    private double adjustmentTargetPosition = Double.NaN;
    private double adjustmentSettlingSeconds;
    private long adjustmentCommandNanos;
    private BallType selectedBallType = BallType.UNKNOWN;
    private SelectionMode selectionMode = SelectionMode.MANUAL;

    /** Uses FORWARD for A and REVERSE for B; readiness thresholds must be supplied explicitly. */
    public Shooter(HardwareMap hardwareMap, double velocityTolerance, double stableTimeSeconds) {
        this(hardwareMap, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                velocityTolerance, stableTimeSeconds);
    }

    /**
     * Configure each direction so intended shooting rotation reports positive velocity.
     * Map flywheelA to Expansion Hub motor/encoder 2 and flywheelB to Control Hub motor/encoder 3.
     *
     * @param velocityTolerance allowed error per motor in encoder ticks per second
     * @param stableTimeSeconds required observed time in tolerance; zero permits immediate readiness
     */
    public Shooter(HardwareMap hardwareMap, DcMotorSimple.Direction directionA,
                   DcMotorSimple.Direction directionB, double velocityTolerance,
                   double stableTimeSeconds) {
        this.velocityTolerance = velocityTolerance;
        this.stableTimeSeconds = stableTimeSeconds;
        flywheelA = initializeMotor(hardwareMap, "flywheelA", directionA);
        flywheelB = initializeMotor(hardwareMap, "flywheelB", directionB);
    }

    /**
     * Adds the optional "shooterAdjust" servo on Expansion Hub servo port 1.
     * Existing constructors remain flywheel-only and never look up this servo.
     * Sets direction without commanding a position; presets require explicit configuration.
     * The settling interval is a time estimate in seconds, not position feedback.
     */
    public Shooter(HardwareMap hardwareMap, DcMotorSimple.Direction directionA,
                   DcMotorSimple.Direction directionB, double velocityTolerance,
                   double stableTimeSeconds, Servo.Direction adjustmentDirection,
                   double adjustmentSettlingSeconds) {
        this(hardwareMap, directionA, directionB, velocityTolerance, stableTimeSeconds);
        adjustmentServo = hardwareMap.get(Servo.class, "shooterAdjust");
        adjustmentServo.setDirection(adjustmentDirection);
        this.adjustmentSettlingSeconds = adjustmentSettlingSeconds;
    }

    /**
     * Commands both motors in encoder ticks per second. Zero stops and clears readiness.
     * Repeating the same target preserves the stability timer.
     */
    public void setTargetVelocity(double ticksPerSecond) {
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

    /** Attempts both zero-velocity commands independently; failures are logged, encoders retained. */
    public void stop() {
        trackingStableTime = false;
        targetVelocity = 0.0;
        try {
            flywheelA.setVelocity(0.0);
        } catch (RuntimeException e) {
            RobotLog.ee("Shooter", e, "Failed to stop flywheelA");
        }
        try {
            flywheelB.setVelocity(0.0);
        } catch (RuntimeException e) {
            RobotLog.ee("Shooter", e, "Failed to stop flywheelB");
        }
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
        if (!(Math.abs(targetVelocity - velocityA) <= velocityTolerance
                && Math.abs(targetVelocity - velocityB) <= velocityTolerance)) {
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

    /** Sets the tolerance in ticks per second; changes restart timing. */
    public void setVelocityTolerance(double velocityTolerance) {
        if (this.velocityTolerance != velocityTolerance) {
            trackingStableTime = false;
        }
        this.velocityTolerance = velocityTolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }

    /** Sets the stability duration in seconds; changes restart timing. */
    public void setStableTimeSeconds(double stableTimeSeconds) {
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
     * Changes leave command history intact; call a mode method to apply a new preset.
     */
    public void configureAdjustmentPositions(double pollen, double nectar) {
        pollenPosition = pollen;
        nectarPosition = nectar;
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
        if (detectedBallType == null) {
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
                || !isAdjustmentSettled()) {
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
     * Sets the settling interval. Changing it re-evaluates elapsed time
     * from the last position command; it does not move the servo or restart that timestamp.
     */
    public void setAdjustmentSettlingSeconds(double seconds) {
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
        if (adjustmentServo == null || Double.isNaN(position)) {
            selectedBallType = BallType.UNKNOWN; // Flywheel-only caller or no target supplied.
            return;
        }
        if (Double.isNaN(adjustmentTargetPosition) || position != adjustmentTargetPosition) {
            adjustmentServo.setPosition(position);
            adjustmentTargetPosition = position;
            adjustmentCommandNanos = System.nanoTime();
        }
        selectedBallType = ballType;
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

}
