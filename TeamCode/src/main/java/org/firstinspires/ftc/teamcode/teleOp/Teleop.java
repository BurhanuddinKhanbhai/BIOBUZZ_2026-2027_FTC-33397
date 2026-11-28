package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;


import org.firstinspires.ftc.teamcode.robot.ShootingCoordinator;
import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/** Robot-centric driving on Gamepad 1 and mechanism controls on Gamepad 2. */
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class Teleop extends OpMode {
    // Competition drive scales.
    private static final double NORMAL_SPEED_SCALE = 1.0;
    private static final double PRECISION_SPEED_SCALE = 0.6;
    // Per-axis threshold in [0.0, 1.0); inputs outside it retain their original magnitude.
    private static final double JOYSTICK_DEADZONE = 0.05;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    // Both motors reverse while the right trigger exceeds this threshold.
    private static final float TRIGGER_THRESHOLD = 0.20f;

    // Manual speed adjustment in encoder ticks/second.
    private static final double MANUAL_FLYWHEEL_MAX_VELOCITY = Double.MAX_VALUE;
    private static final double MANUAL_FLYWHEEL_INCREMENT = 50.0;
    // Initial readiness tolerance matches RedAuto; verify under load on the robot.
    private static final double FLYWHEEL_VELOCITY_TOLERANCE = 75.0;
    private static final double FLYWHEEL_STABLE_TIME_SECONDS = 0.25;


    // ========== LIMELIGHT AIMING ==========
    // Verify pipeline 0 is configured for the hive on the camera.
    private static final int HIVE_PIPELINE = 0;
    private static final long MAX_VISION_AGE_MS = 200;

    private static final double AIM_KP = 0.020;
    private static final double AIM_MAX_TURN = 0.35;
    private static final double AIM_TOLERANCE_DEG = 1.5;
    private static final double AIM_TX_SETPOINT_DEG = 0.0;

    // ========== DISTANCE CALIBRATION ==========
// Measure these on the robot. Inches and degrees.
// Positive camera pitch means tilted upward.
    private static final double CAMERA_HEIGHT_IN = 10.0;
    private static final double TARGET_HEIGHT_IN = 50.0;
    private static final double CAMERA_PITCH_DEG = 35.0;

    // ========== SHOOTING SPEED CALIBRATION ==========
// Enter two measured shooting distances and
// their successful flywheel speeds (ticks/second).
    private static final double NEAR_DISTANCE_IN = Double.NaN;
    private static final double FAR_DISTANCE_IN = Double.NaN;

    private static final double NEAR_SPEED_TPS = Double.NaN;
    private static final double FAR_SPEED_TPS = Double.NaN;


    // Current calibrated shooter adjustment presets.
    private static final double POLLEN_ADJUSTMENT_POSITION = 0.1;
    private static final double NECTAR_ADJUSTMENT_POSITION = 0.0;
    private static final double ADJUSTMENT_SETTLING_SECONDS = .05;
    private static final Servo.Direction ADJUSTMENT_DIRECTION = Servo.Direction.FORWARD;
    // Calibrated with servo 1 FORWARD and servo 2 REVERSE; do not invert these numbers.
    private static final double FLOWER_RAISED_POSITION = 0.3;
    private static final double FLOWER_LOWERED_POSITION = 0.5;

    // Flywheel speed selection is independent of Shooter's ball/servo selection modes.
    private enum FlywheelSpeedMode { AUTO, MANUAL }

    private DriveTrain driveTrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private BallColorSensor ballColorSensor;
    private ShootingCoordinator shootingCoordinator;


    private LimelightVision limelight;
    private double visionDistanceInches = Double.NaN;
    private double lastAutoVisionSpeed = Double.NaN;
    private double visionTx = Double.NaN, visionTy = Double.NaN;
    private String autoSpeedStatus = "UNCALIBRATED";

    private FlowerMechanism flowerMechanism;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private boolean previousBallSelectionButton;
    private boolean adjustmentAvailable;
    private FlywheelSpeedMode flywheelSpeedMode = FlywheelSpeedMode.AUTO;
    private boolean flywheelsEnabled;
    private double manualTargetVelocity;
    private boolean previousY;
    private boolean previousSpeedModeButton;
    private boolean previousDpadUp;
    private boolean previousDpadDown;

    @Override
    public void init() {
        // Retain DriveTrain's motor directions, BRAKE behavior, and encoder-safe motor modes.
        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.setSpeedScale(NORMAL_SPEED_SCALE);
        intake = new Intake(hardwareMap, DcMotorSimple.Direction.REVERSE, INTAKE_POWER);
        transfer = new Transfer(hardwareMap, DcMotorSimple.Direction.REVERSE, TRANSFER_POWER);
        initializeBallSelection();
        initializeFlowerMechanism();

        limelight = new LimelightVision(
                hardwareMap,
                HIVE_PIPELINE,
                MAX_VISION_AGE_MS
        );

// Pipeline selection and camera startup happen once,
// not repeatedly inside the drive control loop.
        limelight.start();

        flywheelSpeedMode = FlywheelSpeedMode.AUTO;
        flywheelsEnabled = false;
        manualTargetVelocity = 0.0;
        previousY = previousSpeedModeButton = previousDpadUp = previousDpadDown = false;
        telemetry.addData("Status", "Drivetrain, intake, transfer, and flywheels initialized");
        telemetry.addData("Speed scale", driveTrain.getSpeedScale());
        telemetry.addData("Flywheel speed status", "AUTO UNCALIBRATED");
        addBallSelectionTelemetry();
        addFlowerTelemetry();

        addVisionTelemetry();

        telemetry.update();
    }

    @Override
    public void start() {
        // Buttons already held at START must be released before a new single-press action.
        previousY = gamepad2.y;
        previousSpeedModeButton = gamepad2.left_bumper;
        previousBallSelectionButton = gamepad2.right_bumper;
        previousDpadUp = gamepad2.dpad_up;
        previousDpadDown = gamepad2.dpad_down;
        previousDpadLeft = gamepad2.dpad_left;
        previousDpadRight = gamepad2.dpad_right;
    }

    @Override
    public void loop() {
        boolean precision = gamepad1.left_bumper;
        driveTrain.setSpeedScale(precision ? PRECISION_SPEED_SCALE : NORMAL_SPEED_SCALE);

        // Positive commands move forward, strafe right, and turn clockwise from above.
        double forward = applyDeadzone(-gamepad1.left_stick_y);
        double strafe = applyDeadzone(gamepad1.left_stick_x);

        double turn = applyDeadzone(gamepad1.right_stick_x);

        // One cached-result update; a getter can become unavailable at the freshness boundary.
        limelight.update();
        visionTx = limelight.getTx();
        visionTy = limelight.getTy();
        boolean targetAvailable = Double.isFinite(visionTx) && Double.isFinite(visionTy);
        if (!targetAvailable) visionTx = visionTy = Double.NaN;
        visionDistanceInches = estimateDistance(visionTy, CAMERA_HEIGHT_IN,
                TARGET_HEIGHT_IN, CAMERA_PITCH_DEG);
        turn = aimingTurn(turn, gamepad1.right_bumper, visionTx);

// Forward and strafe always remain driver-controlled.
// Without a valid target, rotation remains manual.
        driveTrain.drive(forward, strafe, turn);


        // Current competition controls: RT overrides A, B and X.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            // Outtake: both motors reverse at 50%
            intake.setPower(-0.50);
            transfer.setPower(-0.50);

        } else if (gamepad2.a) {
            // Intake and transfer at 100%
            intake.setPower(1.00);
            transfer.setPower(1.00);

        } else if (gamepad2.b) {
            // Intake only at 100%
            intake.setPower(1.00);
            transfer.stop();

        } else if (gamepad2.x) {
            // Intake at 60%, transfer at 15%
            intake.setPower(0.6);
            transfer.setPower(0.15);

        } else {
            intake.stop();
            transfer.stop();
        }

        double flywheelTargetVelocity = updateFlywheels();
        updateBallSelectionMode();
        // Coordinator alone updates color, then shooter selection, then software feed readiness.
        shootingCoordinator.update();
        // Feed-ready already proves at-speed; keep that snapshot instead of polling it again.
        boolean flywheelsAtSpeed = shootingCoordinator.isReadyToFeed() || shooter.isAtSpeed();
        updateFlowerMechanism();

        telemetry.addData("Drive command", "Forward %.2f | Strafe %.2f | Turn %.2f",
                forward, strafe, turn);
        telemetry.addData("Drive mode", precision ? "PRECISION" : "NORMAL");
        telemetry.addData("Speed scale", driveTrain.getSpeedScale());
        telemetry.addData("Front power", "Left %.2f | Right %.2f",
                driveTrain.getFrontLeftPower(), driveTrain.getFrontRightPower());
        telemetry.addData("Rear power", "Left %.2f | Right %.2f",
                driveTrain.getBackLeftPower(), driveTrain.getBackRightPower());
        telemetry.addData("Drive state", driveTrain.getState());
        telemetry.addData("Intake state", intake.getState());
        telemetry.addData("Intake power", intake.getPower());
        telemetry.addData("Transfer state", transfer.getState());
        telemetry.addData("Transfer power", transfer.getPower());
        telemetry.addData("Flywheel enabled request", flywheelsEnabled);
        telemetry.addData("Flywheel speed mode", flywheelSpeedMode);
        telemetry.addData("Flywheel speed status", flywheelSpeedMode == FlywheelSpeedMode.AUTO
                ? "AUTO " + autoSpeedStatus : "MANUAL");
        telemetry.addData("Manual target (ticks/s)", manualTargetVelocity);
        telemetry.addData("Mode target (ticks/s)", flywheelTargetVelocity);
        telemetry.addData("Commanded velocity (ticks/s)", shooter.getTargetVelocity());
        telemetry.addData("Flywheel A (ticks/s)", shooter.getVelocityA());
        telemetry.addData("Flywheel B (ticks/s)", shooter.getVelocityB());
        telemetry.addData("Flywheels at speed", flywheelsAtSpeed);
        addBallSelectionTelemetry();
        addFlowerTelemetry();
        addVisionTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelsEnabled = false;
        // A failed hardware command must not prevent shutdown of the remaining mechanisms.
        try {
            if (driveTrain != null) {
                driveTrain.stop();
            }
        } catch (RuntimeException e) {
            RobotLog.ee("Teleop", e, "Failed to stop drivetrain");
        }
        try {
            if (intake != null) {
                intake.stop();
            }
        } catch (RuntimeException e) {
            RobotLog.ee("Teleop", e, "Failed to stop intake");
        }
        try {
            if (transfer != null) {
                transfer.stop();
            }
        } catch (RuntimeException e) {
            RobotLog.ee("Teleop", e, "Failed to stop transfer");
        }
        try {
            if (shooter != null) {
                shooter.stop();
            }
        } catch (RuntimeException e) {
            RobotLog.ee("Teleop", e, "Failed to stop shooter");
        }

        try {
            if (limelight != null) {
                limelight.stop();
            }
        } catch (RuntimeException e) {
            RobotLog.ee("Teleop", e, "Failed to stop Limelight");
        }

    }

    private void initializeFlowerMechanism() {
        flowerMechanism = new FlowerMechanism(hardwareMap, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        // Storing presets sends no servo position command.
        flowerMechanism.configurePositions(FLOWER_RAISED_POSITION, FLOWER_LOWERED_POSITION);
        previousDpadLeft = previousDpadRight = false;
    }

    private void updateFlowerMechanism() {
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        if (left && !previousDpadLeft && !right) {
            flowerMechanism.lower();
        } else if (right && !previousDpadRight && !left) {
            flowerMechanism.raise();
        }
        // Consume both edges during a conflict; releasing one does not command the held one.
        previousDpadLeft = left;
        previousDpadRight = right;
    }

    private void addFlowerTelemetry() {
        telemetry.addData("Flower servo 1 mapped/status", "%s / %s",
                flowerMechanism.isServo1Available(), flowerMechanism.getServo1Status());
        telemetry.addData("Flower servo 2 mapped/status", "%s / %s",
                flowerMechanism.isServo2Available(), flowerMechanism.getServo2Status());
        telemetry.addData("Flower UP/DOWN calibration", flowerMechanism.getCalibrationStatus());
        telemetry.addData("Flower commanded state", flowerMechanism.getState());
        double target1 = flowerMechanism.getTargetPosition1();
        double target2 = flowerMechanism.getTargetPosition2();
        // Commanded targets only; neither servo supplies physical position feedback.
        telemetry.addData("Flower 1 commanded target", Double.isNaN(target1) ? "UNCOMMANDED" : target1);
        telemetry.addData("Flower 2 commanded target", Double.isNaN(target2) ? "UNCOMMANDED" : target2);
    }

    private void initializeBallSelection() {
        adjustmentAvailable = hardwareMap.tryGet(Servo.class, "shooterAdjust") != null;
        // Keep the existing optional flywheel-only construction path.
        if (adjustmentAvailable) {
            shooter = new Shooter(hardwareMap,
                    DcMotorSimple.Direction.FORWARD,
                    DcMotorSimple.Direction.REVERSE,
                    FLYWHEEL_VELOCITY_TOLERANCE,
                    FLYWHEEL_STABLE_TIME_SECONDS,
                    ADJUSTMENT_DIRECTION,
                    ADJUSTMENT_SETTLING_SECONDS);
            shooter.configureAdjustmentPositions(POLLEN_ADJUSTMENT_POSITION, NECTAR_ADJUSTMENT_POSITION);
        } else {
            shooter = new Shooter(hardwareMap, FLYWHEEL_VELOCITY_TOLERANCE,
                    FLYWHEEL_STABLE_TIME_SECONDS);
        }
        previousBallSelectionButton = false;
        shooter.setAutoMode(); // No position command, even with a fully calibrated servo.

        // Load only validated Fast Color references; missing/corrupt data leaves AUTO UNKNOWN.
        ballColorSensor = new BallColorSensor(hardwareMap);
        shootingCoordinator = new ShootingCoordinator(shooter, ballColorSensor);
    }

    private void updateBallSelectionMode() {
        boolean button = gamepad2.right_bumper;
        if (button && !previousBallSelectionButton) {
            switch (shooter.getSelectionMode()) {
                case AUTO:
                    shooter.setPollenMode();
                    break;
                case MANUAL_POLLEN:
                    shooter.setNectarMode();
                    break;
                default:
                    shooter.setAutoMode();
                    break;
            }
        }
        previousBallSelectionButton = button;
    }

    private void addBallSelectionTelemetry() {
        telemetry.addData("Ball-selection mode", shooter.getSelectionMode());
        telemetry.addData("Detected ball type", shootingCoordinator.getDetectedBallType());
        telemetry.addData("Requested ball type", getRequestedBallType());
        telemetry.addData("Applied ball type", shooter.getSelectedBallType());
        telemetry.addData("Color sensor mapped", ballColorSensor.isAvailable());
        telemetry.addData("Color calibration", ballColorSensor.getCalibrationStatus());
        telemetry.addData("Color sensor status", ballColorSensor.getStatus());
        // Cached values only: the coordinator owns the single update/read for this loop.
        telemetry.addData("Color RGB fractions", "%.3f / %.3f / %.3f",
                ballColorSensor.getRedFraction(), ballColorSensor.getGreenFraction(), ballColorSensor.getBlueFraction());
        telemetry.addData("Color distance / gate (cm)", "%.2f / %.2f (NaN = color-only)",
                ballColorSensor.getDistanceCm(), ballColorSensor.getGateDistanceCm());
        telemetry.addData("Color error / margin", "%.3f / %.3f",
                ballColorSensor.getBestColorError(), ballColorSensor.getColorMargin());
        telemetry.addData("Color confirm readings", ballColorSensor.getConsecutiveReadings());
        telemetry.addData("Color read / confirmation (ms)", "%.2f / %.2f",
                ballColorSensor.getReadTimeMs(), ballColorSensor.getConfirmationTimeMs());
        telemetry.addData("Color support", ballColorSensor.isRedCalibrated()
                ? "Yellow pollen / blue + red nectar" : "Yellow pollen / blue nectar; red nectar UNCALIBRATED");
        telemetry.addData("Shooter servo mapped", adjustmentAvailable);
        telemetry.addData("Shooter servo presets", "Pollen %.3f / Nectar %.3f",
                POLLEN_ADJUSTMENT_POSITION, NECTAR_ADJUSTMENT_POSITION);
        double position = shooter.getAdjustmentTargetPosition();
        telemetry.addData("Shooter servo command", Double.isNaN(position) ? "UNCOMMANDED" : position);
        telemetry.addData("Shooter servo settled", shooter.isAdjustmentSettled());
        // Software readiness only; does not confirm safe staging or operate the manual transfer.
        telemetry.addData("Shooter ready to feed", shootingCoordinator.isReadyToFeed());
    }

    /** Driver intent for telemetry, not confirmation that a servo command could be applied. */
    private BallColorSensor.BallType getRequestedBallType() {
        if (shooter.getSelectionMode() == Shooter.SelectionMode.MANUAL_POLLEN) {
            return BallColorSensor.BallType.POLLEN;
        }
        if (shooter.getSelectionMode() == Shooter.SelectionMode.MANUAL_NECTAR) {
            return BallColorSensor.BallType.NECTAR;
        }
        return shootingCoordinator.getDetectedBallType();
    }

    /** Updates rising edges and commands velocity; returns the mode target before the ON/OFF gate. */
    private double updateFlywheels() {
        boolean y = gamepad2.y;
        boolean speedModeButton = gamepad2.left_bumper;
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        if (y && !previousY) {
            flywheelsEnabled = !flywheelsEnabled;
        }
        if (speedModeButton && !previousSpeedModeButton) {
            flywheelSpeedMode = flywheelSpeedMode == FlywheelSpeedMode.AUTO
                    ? FlywheelSpeedMode.MANUAL : FlywheelSpeedMode.AUTO;
        }

        if (flywheelSpeedMode == FlywheelSpeedMode.MANUAL) {
            // Ignore conflicting D-pad directions; adjustments are allowed while OFF.
            if (up && !previousDpadUp && !down) {
                manualTargetVelocity = Math.min(MANUAL_FLYWHEEL_MAX_VELOCITY,
                        manualTargetVelocity + MANUAL_FLYWHEEL_INCREMENT);
            } else if (down && !previousDpadDown && !up) {
                manualTargetVelocity = Math.max(0.0,
                        manualTargetVelocity - MANUAL_FLYWHEEL_INCREMENT);
            }
        }
        // Track edges even in AUTO; a held button is not a new press.
        previousY = y;
        previousSpeedModeButton = speedModeButton;
        previousDpadUp = up;
        previousDpadDown = down;

        // Drop AUTO history in MANUAL; returning to AUTO uses the current observation.
        if (flywheelSpeedMode == FlywheelSpeedMode.MANUAL) lastAutoVisionSpeed = Double.NaN;

        double targetVelocity = flywheelSpeedMode == FlywheelSpeedMode.MANUAL
                ? manualTargetVelocity
                : getDistanceBasedFlywheelSpeed();

        shooter.setTargetVelocity(flywheelsEnabled ? targetVelocity : 0.0);
        return targetVelocity;
    }


    private void addVisionTelemetry() {
        telemetry.addData("Limelight / pipeline", "%s / %d", limelight.getStatus(), limelight.getPipelineIndex());
        telemetry.addData("AprilTag ID",
                limelight.getSelectedTagId() == -1
                        ? "NONE"
                        : limelight.getSelectedTagId());
        telemetry.addData("Vision tx / ty (degrees)", "%.2f / %.2f", visionTx, visionTy);
        telemetry.addData("Vision age (ms)", limelight.getResultAgeMillis());
        telemetry.addData("Vision distance (inches)", Double.isFinite(visionDistanceInches)
                ? String.format("%.1f", visionDistanceInches) : "UNAVAILABLE / UNCALIBRATED");
        telemetry.addData("Auto aim", !gamepad1.right_bumper ? "OFF"
                : !Double.isFinite(visionTx) ? "NO TARGET - MANUAL"
                : Math.abs(visionTx - AIM_TX_SETPOINT_DEG) <= AIM_TOLERANCE_DEG ? "ALIGNED" : "TRACKING");
        telemetry.addData("Vision flywheel target", flywheelSpeedMode == FlywheelSpeedMode.MANUAL
                ? "MANUAL MODE" : autoSpeedStatus + " / " + (Double.isFinite(lastAutoVisionSpeed) ? lastAutoVisionSpeed : 0.0));
    }

    private static double aimingTurn(double manual, boolean held, double tx) {
        if (!held || !Double.isFinite(tx)) return manual;
        double error = tx - AIM_TX_SETPOINT_DEG;
        // Positive tx is right; positive drivetrain turn is clockwise. Verify mounting on robot.
        return Math.abs(error) <= AIM_TOLERANCE_DEG ? 0.0
                : Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, AIM_KP * error));
    }

    /** Horizontal camera-to-target distance; calibrate near the aiming crosshair. */
    private static double estimateDistance(double ty, double cameraHeight, double targetHeight, double pitch) {
        double angle = pitch + ty;
        if (!Double.isFinite(ty) || !Double.isFinite(cameraHeight) || !Double.isFinite(targetHeight)
                || !Double.isFinite(pitch) || cameraHeight < 0 || targetHeight < 0
                || !Double.isFinite(angle) || Math.abs(angle) >= 90.0) return Double.NaN;
        double height = targetHeight - cameraHeight;
        double tangent = Math.tan(Math.toRadians(angle));
        if (Math.abs(tangent) < 0.0001 || Math.abs(height) < 0.01) return Double.NaN;
        double distance = height / tangent;
        return Double.isFinite(distance) && distance > 0 ? distance : Double.NaN;
    }

    private static boolean speedCalibrationValid(double near, double far, double nearSpeed, double farSpeed) {
        return Double.isFinite(near) && Double.isFinite(far) && near > 0 && far > near
                && Double.isFinite(nearSpeed) && Double.isFinite(farSpeed) && nearSpeed > 0 && farSpeed > 0;
    }

    private static double interpolatedSpeed(double distance, double near, double far, double nearSpeed, double farSpeed) {
        if (!Double.isFinite(distance) || distance <= 0 || !speedCalibrationValid(near, far, nearSpeed, farSpeed))
            return Double.NaN;
        double fraction = Math.max(0.0, Math.min(1.0, (distance - near) / (far - near)));
        return nearSpeed + fraction * (farSpeed - nearSpeed);
    }

    private static double stabilizedSpeed(double calculated, double previous, double nearSpeed, double farSpeed) {
        // Clamp AFTER rounding so endpoint calibration remains the actual command bounds.
        double quantized = Math.max(Math.min(nearSpeed, farSpeed), Math.min(Math.max(nearSpeed, farSpeed),
                Math.rint(calculated / 50.0) * 50.0));
        return !Double.isFinite(previous) || Math.abs(quantized - previous) >= 75.0 ? quantized : previous;
    }

    private double getDistanceBasedFlywheelSpeed() {
        if (!Double.isFinite(CAMERA_HEIGHT_IN) || !Double.isFinite(TARGET_HEIGHT_IN)
                || !Double.isFinite(CAMERA_PITCH_DEG)
                || !speedCalibrationValid(NEAR_DISTANCE_IN, FAR_DISTANCE_IN, NEAR_SPEED_TPS, FAR_SPEED_TPS)) {
            autoSpeedStatus = "UNCALIBRATED";
        } else if (!Double.isFinite(visionTx) || !Double.isFinite(visionTy)) {
            autoSpeedStatus = "NO TARGET";
        } else if (!Double.isFinite(visionDistanceInches)) {
            autoSpeedStatus = "INVALID GEOMETRY";
        } else {
            double calculated = interpolatedSpeed(visionDistanceInches, NEAR_DISTANCE_IN, FAR_DISTANCE_IN,
                    NEAR_SPEED_TPS, FAR_SPEED_TPS);
            lastAutoVisionSpeed = stabilizedSpeed(calculated, lastAutoVisionSpeed, NEAR_SPEED_TPS, FAR_SPEED_TPS);
            autoSpeedStatus = visionDistanceInches < NEAR_DISTANCE_IN || visionDistanceInches > FAR_DISTANCE_IN
                    ? "ENDPOINT CLAMP" : "TRACKING";
            return lastAutoVisionSpeed;
        }
        lastAutoVisionSpeed = Double.NaN;
        return 0.0;
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) <= JOYSTICK_DEADZONE ? 0.0 : value;
    }
}
