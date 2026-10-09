package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Controls the rubber-band transfer motor; both ball-sizing flaps are passive. */
public class Transfer {
    private final DcMotorEx motor;
    private double defaultPower;
    private double power;

    /** Uses FORWARD direction and full power for feed/reverse commands. */
    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, DcMotorSimple.Direction.FORWARD, 1.0);
    }

    /**
     * Initialize before establishing any shared odometry encoder baseline.
     * Positive power feeds toward the shooter with the supplied motor direction.
     * Configure "transfer" on Expansion Hub motor port 2.
     *
     * @param defaultPower feed/reverse power magnitude, from 0.0 to 1.0
     */
    public Transfer(HardwareMap hardwareMap, DcMotorSimple.Direction direction,
                    double defaultPower) {
        if (direction == null) {
            throw new IllegalArgumentException("Transfer direction must not be null");
        }
        setDefaultPower(defaultPower);
        motor = hardwareMap.get(DcMotorEx.class, "transfer");
        motor.setPower(0.0);
        motor.setDirection(direction);
        // The encoder input may belong to an odometry pod. Never reset it.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /** Feeds toward the shooter at the configured default power. */
    public void feed() {
        setPower(defaultPower);
    }

    /** Runs away from the shooter at the configured default power. */
    public void reverse() {
        setPower(-defaultPower);
    }

    public void stop() {
        setPower(0.0);
    }

    /**
     * Immediately applies signed power, clipped to [-1.0, 1.0].
     * Positive feeds, negative reverses, and zero stops.
     */
    public void setPower(double power) {
        if (Double.isNaN(power) || Double.isInfinite(power)) {
            throw new IllegalArgumentException("Transfer power must be finite");
        }
        double clippedPower = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(clippedPower);
        this.power = clippedPower;
    }

    /** Changes the magnitude used by subsequent feed() and reverse() calls. */
    public void setDefaultPower(double defaultPower) {
        if (Double.isNaN(defaultPower) || Double.isInfinite(defaultPower)
                || defaultPower < 0.0 || defaultPower > 1.0) {
            throw new IllegalArgumentException("Default transfer power must be between 0.0 and 1.0");
        }
        this.defaultPower = defaultPower;
    }

    public double getDefaultPower() {
        return defaultPower;
    }

    /** Returns the last commanded power for telemetry, not measured motor speed. */
    public double getPower() {
        return power;
    }

    /** Returns the commanded operating state for telemetry. */
    public String getState() {
        if (power > 0.0) {
            return "FEEDING";
        }
        if (power < 0.0) {
            return "REVERSING";
        }
        return "STOPPED";
    }
}
