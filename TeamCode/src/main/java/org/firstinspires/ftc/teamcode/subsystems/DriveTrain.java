package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Robot-centric control for a conventional X-pattern mecanum drivetrain. */
public class DriveTrain {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private double speedScale = 1.0;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    /**
     * Assumes mirrored mounting: left motors REVERSE, right motors FORWARD.
     * Uses BRAKE and full speed scaling. Belt routing may require direction overrides.
     */
    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Each configured motor must roll its wheel forward at positive power.
     * Directions are ordered front-left, front-right, back-left, back-right.
     * Initialize before establishing shared odometry encoder baselines.
     * Hardware names identify wheel locations regardless of which hub is on each side.
     */
    public DriveTrain(HardwareMap hardwareMap,
                      DcMotorSimple.Direction frontLeftDirection,
                      DcMotorSimple.Direction frontRightDirection,
                      DcMotorSimple.Direction backLeftDirection,
                      DcMotorSimple.Direction backRightDirection,
                      DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (hardwareMap == null || frontLeftDirection == null || frontRightDirection == null
                || backLeftDirection == null || backRightDirection == null) {
            throw new IllegalArgumentException("Hardware map and motor directions must not be null");
        }
        validateZeroPowerBehavior(zeroPowerBehavior);
        frontLeft = initializeMotor(hardwareMap, "frontLeft", frontLeftDirection, zeroPowerBehavior);
        frontRight = initializeMotor(hardwareMap, "frontRight", frontRightDirection, zeroPowerBehavior);
        backLeft = initializeMotor(hardwareMap, "backLeft", backLeftDirection, zeroPowerBehavior);
        backRight = initializeMotor(hardwareMap, "backRight", backRightDirection, zeroPowerBehavior);
        this.zeroPowerBehavior = zeroPowerBehavior;
        stop();
    }

    /**
     * Positive forward moves toward the robot's front, positive strafe moves right,
     * and positive turn rotates clockwise when viewed from above.
     * Finite inputs are clipped to [-1.0, 1.0] before mixing. Any nonfinite input stops
     * all motors and returns. Wheel powers are normalized together, then speed-scaled.
     */
    public void drive(double forward, double strafe, double turn) {
        if (!isFinite(forward) || !isFinite(strafe) || !isFinite(turn)) {
            stop();
            return;
        }
        forward = Math.max(-1.0, Math.min(1.0, forward));
        strafe = Math.max(-1.0, Math.min(1.0, strafe));
        turn = Math.max(-1.0, Math.min(1.0, turn));

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        double denominator = Math.max(1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))));
        applyPowers((fl / denominator) * speedScale, (fr / denominator) * speedScale,
                (bl / denominator) * speedScale, (br / denominator) * speedScale);
    }

    /** Stops all motors without changing motor modes or encoder counts. */
    public void stop() {
        applyPowers(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Sets scaling for subsequent drive() calls, from 0.0 to 1.0.
     * Invalid values stop the drivetrain and throw IllegalArgumentException.
     */
    public void setSpeedScale(double speedScale) {
        if (!isFinite(speedScale) || speedScale < 0.0 || speedScale > 1.0) {
            stop();
            throw new IllegalArgumentException("Drive speed scale must be between 0.0 and 1.0");
        }
        this.speedScale = speedScale;
    }

    public double getSpeedScale() {
        return speedScale;
    }

    /** Applies BRAKE or FLOAT to all motors without changing encoder configuration. */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        validateZeroPowerBehavior(zeroPowerBehavior);
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    /** Returns last commanded power, not measured wheel speed. */
    public double getFrontLeftPower() {
        return frontLeftPower;
    }

    /** Returns last commanded power, not measured wheel speed. */
    public double getFrontRightPower() {
        return frontRightPower;
    }

    /** Returns last commanded power, not measured wheel speed. */
    public double getBackLeftPower() {
        return backLeftPower;
    }

    /** Returns last commanded power, not measured wheel speed. */
    public double getBackRightPower() {
        return backRightPower;
    }

    /** Reports commanded state; STOPPED does not confirm the robot has stopped moving. */
    public String getState() {
        return frontLeftPower == 0.0 && frontRightPower == 0.0
                && backLeftPower == 0.0 && backRightPower == 0.0 ? "STOPPED" : "DRIVING";
    }

    private void applyPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontLeftPower = fl;
        frontRight.setPower(fr);
        frontRightPower = fr;
        backLeft.setPower(bl);
        backLeftPower = bl;
        backRight.setPower(br);
        backRightPower = br;
    }

    private static DcMotorEx initializeMotor(HardwareMap hardwareMap, String name,
                                            DcMotorSimple.Direction direction,
                                            DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setPower(0.0);
        motor.setDirection(direction);
        // Encoder inputs may belong to odometry pods; never reset or use them for motor control.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        return motor;
    }

    private static void validateZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (behavior != DcMotor.ZeroPowerBehavior.BRAKE && behavior != DcMotor.ZeroPowerBehavior.FLOAT) {
            throw new IllegalArgumentException("Drive zero-power behavior must be BRAKE or FLOAT");
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
