package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Commands flower servo targets directly; reported positions are commands, not feedback. */
public class FlowerMechanism {
    private final Servo servo;
    private final Servo servo2;
    private final Servo.Direction direction1;
    private final Servo.Direction direction2;
    private double raisedPosition = Double.NaN;
    private double loweredPosition = Double.NaN;
    private double targetPosition = Double.NaN;
    private double targetPosition2 = Double.NaN;
    private String state = "UNCOMMANDED";

    /** Single-servo ownership, used by autonomous's separate servo-2 control. */
    public FlowerMechanism(HardwareMap hardwareMap) {
        this(hardwareMap, Servo.Direction.FORWARD);
    }

    public FlowerMechanism(HardwareMap hardwareMap, Servo.Direction direction) {
        servo = hardwareMap.get(Servo.class, "flowerServo");
        servo2 = null;
        direction1 = direction;
        direction2 = null;
        servo.setDirection(direction);
    }

    /** Both linked servos, with direction reversal applied by the SDK only. */
    public FlowerMechanism(HardwareMap hardwareMap, Servo.Direction direction1, Servo.Direction direction2) {
        servo = hardwareMap.get(Servo.class, "flowerServo");
        servo2 = hardwareMap.get(Servo.class, "flowerServo2");
        this.direction1 = direction1;
        this.direction2 = direction2;
        servo.setDirection(direction1);
        servo2.setDirection(direction2);
    }

    /** Stores endpoints without moving either servo. */
    public void configurePositions(double raisedPosition, double loweredPosition) {
        this.raisedPosition = raisedPosition;
        this.loweredPosition = loweredPosition;
    }

    public void raise() {
        if (Double.isNaN(raisedPosition)) return; // No target has been supplied yet.
        setPosition(raisedPosition);
        state = "RAISE_COMMANDED";
    }

    public void lower() {
        if (Double.isNaN(loweredPosition)) return;
        setPosition(loweredPosition);
        state = "LOWER_COMMANDED";
    }

    public void setPosition(double position) {
        servo.setPosition(position);
        targetPosition = position;
        if (servo2 != null) {
            servo2.setPosition(position);
            targetPosition2 = position;
        }
        state = "POSITION_COMMANDED";
    }

    public double getTargetPosition() { return targetPosition; }
    public double getTargetPosition1() { return targetPosition; }
    public double getTargetPosition2() { return targetPosition2; }
    public boolean isServo1Available() { return servo != null; }
    public boolean isServo2Available() { return servo2 != null; }
    public boolean arePositionsConfigured() {
        return !Double.isNaN(raisedPosition) && !Double.isNaN(loweredPosition);
    }
    public String getCalibrationStatus() {
        return arePositionsConfigured() ? "UP " + raisedPosition + " / DOWN " + loweredPosition : "UNCONFIGURED";
    }
    public String getServo1Status() { return "MAPPED / " + direction1; }
    public String getServo2Status() { return servo2 == null ? "NOT OWNED" : "MAPPED / " + direction2; }
    public boolean isReady() { return arePositionsConfigured(); }
    public String getState() { return state; }
}
