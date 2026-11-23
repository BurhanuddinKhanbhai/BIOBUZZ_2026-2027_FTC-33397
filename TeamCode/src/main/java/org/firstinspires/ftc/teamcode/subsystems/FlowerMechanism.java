package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/** Commands flower servos without physical position feedback or a movement rate limiter. */
public class FlowerMechanism {
    private final Servo servo;
    private final Servo servo2;
    private final boolean linkedArm;
    private double raisedPosition;
    private double loweredPosition;
    private boolean positionsConfigured;
    private boolean hardwareFault;
    private double targetPosition = Double.NaN;
    private double targetPosition2 = Double.NaN;
    private String state = "UNCOMMANDED";
    private String servo1Status;
    private String servo2Status = "NOT OWNED (single-servo constructor)";
    private String calibrationStatus = "UNCONFIGURED";

    /** Legacy single-servo ownership, retained for autonomous's separate servo-2 control. */
    public FlowerMechanism(HardwareMap hardwareMap) {
        this(hardwareMap, Servo.Direction.FORWARD);
    }

    /**
     * Configures direction without commanding a position.
     * Single-servo compatibility constructor: "flowerServo" on Control Hub servo port 0.
     * Competition TeleOp uses the two-direction constructor to own the complete linked arm.
     */
    public FlowerMechanism(HardwareMap hardwareMap, Servo.Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException("Flower servo direction must not be null");
        }
        servo = hardwareMap.get(Servo.class, "flowerServo");
        servo.setDirection(direction);
        servo2 = null;
        linkedArm = false;
        servo1Status = "MAPPED / " + direction;
    }

    /**
     * Owns both servos: Control Hub servo 0 and Expansion Hub servo 0. Missing devices
     * disable linked movement. Direction reversal is applied here only; targets are not inverted.
     * Neither lookup nor direction configuration sends a position command.
     */
    public FlowerMechanism(HardwareMap hardwareMap, Servo.Direction direction1, Servo.Direction direction2) {
        if (hardwareMap == null || direction1 == null || direction2 == null) {
            throw new IllegalArgumentException("Hardware map and both flower directions are required");
        }
        linkedArm = true;
        servo = findServo(hardwareMap, "flowerServo");
        servo2 = findServo(hardwareMap, "flowerServo2");
        servo1Status = configureDirection(servo, direction1, "flowerServo");
        servo2Status = configureDirection(servo2, direction2, "flowerServo2");
        if (servo != null && servo == servo2) {
            hardwareFault = true;
            servo1Status = servo2Status = "INVALID MAPPING - same servo device";
        }
    }

    /**
     * Sets calibrated raised/lowered targets in the configured servo direction.
     * Both positions must be finite and within [0.0, 1.0]. This does not move the servo.
     */
    public void configurePositions(double raisedPosition, double loweredPosition) {
        // A failed replacement calibration must not leave old linked-arm presets enabled.
        positionsConfigured = false;
        calibrationStatus = "INVALID UP/DOWN - positions must be finite and in [0, 1]";
        validatePosition(raisedPosition);
        validatePosition(loweredPosition);
        this.raisedPosition = raisedPosition;
        this.loweredPosition = loweredPosition;
        positionsConfigured = true;
        calibrationStatus = "CONFIGURED: UP " + raisedPosition + " / DOWN " + loweredPosition;
    }

    /** Commands the raised target; requires explicit position configuration first. */
    public void raise() {
        requireConfiguredPositions();
        setPosition(raisedPosition);
        state = "RAISE_COMMANDED";
    }

    /** Commands the lowered target; requires explicit position configuration first. */
    public void lower() {
        requireConfiguredPositions();
        setPosition(loweredPosition);
        state = "LOWER_COMMANDED";
    }

    /**
     * Commands a finite position within [0.0, 1.0]; invalid values are rejected.
     * Linked-arm commands require BOTH mapped servos and valid configured endpoints.
     * Legacy single-servo callers may still command an explicit position before calibration.
     * A valid SDK position is not necessarily safe for the attached mechanism.
     */
    public void setPosition(double position) {
        validatePosition(position);
        if (linkedArm) requireConfiguredPositions();
        if (hardwareFault) throw new IllegalStateException("Flower hardware fault; reinitialize before commanding");
        try {
            servo.setPosition(position);
            targetPosition = position;
        } catch (RuntimeException e) {
            hardwareFault = true;
            servo1Status = "COMMAND FAILED: " + e.getMessage();
            state = "COMMAND_FAILED - physical position unknown";
            throw e;
        }
        if (linkedArm) {
            try {
                // Same logical target, same caller iteration. REVERSE already handles shaft direction.
                servo2.setPosition(position);
                targetPosition2 = position;
            } catch (RuntimeException e) {
                hardwareFault = true;
                servo2Status = "COMMAND FAILED: " + e.getMessage();
                state = "PARTIAL_COMMAND_FAILED - physical positions unknown";
                // No guessed rollback or automatic retries on the mechanically linked arm.
                throw e;
            }
        }
        state = "POSITION_COMMANDED";
    }

    /** Returns the last commanded target, or NaN before any command; never actual position. */
    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetPosition1() { return targetPosition; }
    public double getTargetPosition2() { return targetPosition2; }
    public boolean isServo1Available() { return servo != null; }
    public boolean isServo2Available() { return servo2 != null; }
    public boolean arePositionsConfigured() { return positionsConfigured; }
    public String getCalibrationStatus() { return calibrationStatus; }
    public String getServo1Status() { return servo1Status; }
    public String getServo2Status() { return servo2Status; }
    public boolean isReady() {
        return positionsConfigured && !hardwareFault && servo != null && (!linkedArm || servo2 != null);
    }

    /** Returns the last command state, not confirmation that the mechanism has arrived. */
    public String getState() {
        return state;
    }

    private void requireConfiguredPositions() {
        if (!positionsConfigured) {
            throw new IllegalStateException("Configure flower positions before calling raise() or lower()");
        }
        if (!isReady()) {
            throw new IllegalStateException("Flower movement disabled: " + servo1Status + " / " + servo2Status);
        }
    }

    private Servo findServo(HardwareMap hardwareMap, String name) {
        try { return hardwareMap.tryGet(Servo.class, name); }
        catch (RuntimeException e) {
            hardwareFault = true;
            RobotLog.ee("FlowerMechanism", e, "Cannot map " + name);
            return null;
        }
    }

    private String configureDirection(Servo device, Servo.Direction direction, String name) {
        if (device == null) return "MISSING / UNAVAILABLE";
        try {
            device.setDirection(direction);
            return "MAPPED / " + direction;
        } catch (RuntimeException e) {
            hardwareFault = true;
            RobotLog.ee("FlowerMechanism", e, "Cannot configure direction for " + name);
            return "DIRECTION ERROR: " + e.getMessage();
        }
    }

    private static void validatePosition(double position) {
        if (Double.isNaN(position) || Double.isInfinite(position)
                || position < 0.0 || position > 1.0) {
            throw new IllegalArgumentException("Flower servo position must be between 0.0 and 1.0");
        }
    }
}
