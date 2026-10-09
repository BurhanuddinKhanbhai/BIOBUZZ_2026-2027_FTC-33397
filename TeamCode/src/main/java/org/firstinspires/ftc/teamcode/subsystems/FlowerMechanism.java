package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Controls the flower mechanism servo without physical position feedback. */
public class FlowerMechanism {
    private final Servo servo;
    private double raisedPosition;
    private double loweredPosition;
    private boolean positionsConfigured;
    private double targetPosition = Double.NaN;
    private String state = "UNCOMMANDED";

    /** Uses FORWARD direction without commanding a position. */
    public FlowerMechanism(HardwareMap hardwareMap) {
        this(hardwareMap, Servo.Direction.FORWARD);
    }

    /**
     * Configures direction without commanding a position.
     * Configure "flowerServo" on Control Hub servo port 1.
     */
    public FlowerMechanism(HardwareMap hardwareMap, Servo.Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException("Flower servo direction must not be null");
        }
        servo = hardwareMap.get(Servo.class, "flowerServo");
        servo.setDirection(direction);
    }

    /**
     * Sets calibrated raised/lowered targets in the configured servo direction.
     * Both positions must be finite and within [0.0, 1.0]. This does not move the servo.
     */
    public void configurePositions(double raisedPosition, double loweredPosition) {
        validatePosition(raisedPosition);
        validatePosition(loweredPosition);
        this.raisedPosition = raisedPosition;
        this.loweredPosition = loweredPosition;
        positionsConfigured = true;
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
     * Available before configuration for manual calibration; it does not configure presets.
     * A valid SDK position is not necessarily safe for the attached mechanism.
     */
    public void setPosition(double position) {
        validatePosition(position);
        servo.setPosition(position);
        targetPosition = position;
        state = "POSITION_COMMANDED";
    }

    /** Returns the last commanded target, or NaN before any command; never actual position. */
    public double getTargetPosition() {
        return targetPosition;
    }

    /** Returns the last command state, not confirmation that the mechanism has arrived. */
    public String getState() {
        return state;
    }

    private void requireConfiguredPositions() {
        if (!positionsConfigured) {
            throw new IllegalStateException("Configure flower positions before calling raise() or lower()");
        }
    }

    private static void validatePosition(double position) {
        if (Double.isNaN(position) || Double.isInfinite(position)
                || position < 0.0 || position > 1.0) {
            throw new IllegalArgumentException("Flower servo position must be between 0.0 and 1.0");
        }
    }
}
