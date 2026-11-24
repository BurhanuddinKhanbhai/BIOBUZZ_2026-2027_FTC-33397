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

    /** Uses REVERSE direction and full power for feed/reverse commands. */
    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, DcMotorSimple.Direction.REVERSE, 1.0);
    }

    /**
     * Initialize before establishing any shared odometry encoder baseline.
     * Positive power feeds toward the shooter with the supplied motor direction.
     * Configure "transfer" on Expansion Hub motor port 3.
     *
     * @param defaultPower feed/reverse power magnitude, from 0.0 to 1.0
     */
    public Transfer(HardwareMap hardwareMap, DcMotorSimple.Direction direction,
                    double defaultPower) {
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
     * Passes signed power to the FTC SDK.
     * Positive feeds, negative reverses, and zero stops.
     */
    public void setPower(double power) {
        motor.setPower(power);
        this.power = power;
    }

    /** Changes the magnitude used by subsequent feed() and reverse() calls. */
    public void setDefaultPower(double defaultPower) {
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
