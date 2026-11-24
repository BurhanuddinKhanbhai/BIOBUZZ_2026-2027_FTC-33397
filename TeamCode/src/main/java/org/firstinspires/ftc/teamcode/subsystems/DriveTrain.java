package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

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
     * Uses the current mounting directions: left FORWARD, right REVERSE.
     * Uses BRAKE and full speed scaling. Belt routing may require direction overrides.
     */
    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
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
     * Wheel powers are normalized together, then speed-scaled.
     */
    public void drive(double forward, double strafe, double turn) {
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

    /** Attempts every motor stop independently, logging failures without resetting encoders. */
    public void stop() {
        try {
            frontLeft.setPower(0.0);
            frontLeftPower = 0.0;
        } catch (RuntimeException e) {
            RobotLog.ee("DriveTrain", e, "Failed to stop frontLeft");
        }
        try {
            frontRight.setPower(0.0);
            frontRightPower = 0.0;
        } catch (RuntimeException e) {
            RobotLog.ee("DriveTrain", e, "Failed to stop frontRight");
        }
        try {
            backLeft.setPower(0.0);
            backLeftPower = 0.0;
        } catch (RuntimeException e) {
            RobotLog.ee("DriveTrain", e, "Failed to stop backLeft");
        }
        try {
            backRight.setPower(0.0);
            backRightPower = 0.0;
        } catch (RuntimeException e) {
            RobotLog.ee("DriveTrain", e, "Failed to stop backRight");
        }
    }

    /** Sets scaling for subsequent drive() calls. */
    public void setSpeedScale(double speedScale) {
        this.speedScale = speedScale;
    }

    public double getSpeedScale() {
        return speedScale;
    }

    /** Applies BRAKE or FLOAT to all motors without changing encoder configuration. */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
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

}
