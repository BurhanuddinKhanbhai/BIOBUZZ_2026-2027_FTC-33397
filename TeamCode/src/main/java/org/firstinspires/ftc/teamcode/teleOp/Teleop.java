package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    // Initial limits for an untested robot; tune within [0.0, 1.0] after hardware validation.
    private static final double NORMAL_SPEED_SCALE = 0.40;
    private static final double PRECISION_SPEED_SCALE = 0.20;
    // Per-axis threshold in [0.0, 1.0); inputs outside it retain their original magnitude.
    private static final double JOYSTICK_DEADZONE = 0.05;
    private static final double INTAKE_POWER = 0.35;
    private static final double TRANSFER_POWER = 0.35;
    // Reverse activates while the corresponding trigger exceeds this threshold.
    private static final float TRIGGER_THRESHOLD = 0.20f;

    // Encoder ticks/second. Leave zero until hardware-appropriate limits are explicitly set.
    private static final double MANUAL_FLYWHEEL_MAX_VELOCITY = 0.0;
    private static final double MANUAL_FLYWHEEL_INCREMENT = 0.0;
    // Uncalibrated readiness settings: zero tolerance requires an exact velocity match.
    private static final double FLYWHEEL_VELOCITY_TOLERANCE = 0.0;
    private static final double FLYWHEEL_STABLE_TIME_SECONDS = 0.25;

    // Unset until measured on the robot. NaN is never passed as a servo command or timing value.
    private static final double POLLEN_ADJUSTMENT_POSITION = Double.NaN;
    private static final double NECTAR_ADJUSTMENT_POSITION = Double.NaN;
    private static final double ADJUSTMENT_SETTLING_SECONDS = Double.NaN;
    private static final Servo.Direction ADJUSTMENT_DIRECTION = Servo.Direction.FORWARD;
    private static final double COLOR_STABLE_TIME_SECONDS = Double.NaN;
    private static final BallColorSensor.ColorProfile POLLEN_COLOR_PROFILE = null;
    private static final BallColorSensor.ColorProfile NECTAR_COLOR_PROFILE = null;
    private static final long COLOR_MINIMUM_SIGNAL = -1;
    // Leave unset until safe flower travel positions have been measured on the mechanism.
    private static final double FLOWER_RAISED_POSITION = Double.NaN;
    private static final double FLOWER_LOWERED_POSITION = Double.NaN;

    // Flywheel speed selection is independent of Shooter's ball/servo selection modes.
    private enum FlywheelSpeedMode { AUTO, MANUAL }

    private DriveTrain driveTrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private BallColorSensor ballColorSensor;
    private ShootingCoordinator shootingCoordinator;
    private FlowerMechanism flowerMechanism;
    private boolean flowerPositionsConfigured;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    // Retains driver intent even when the optional servo cannot yet apply a manual mode.
    private Shooter.SelectionMode ballSelectionMode = Shooter.SelectionMode.AUTO;
    private boolean previousBallSelectionButton;
    private boolean adjustmentAvailable;
    private boolean adjustmentCalibrated;
    private boolean colorSensorAvailable;
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
        intake = new Intake(hardwareMap, DcMotorSimple.Direction.FORWARD, INTAKE_POWER);
        transfer = new Transfer(hardwareMap, DcMotorSimple.Direction.FORWARD, TRANSFER_POWER);
        initializeBallSelection();
        initializeFlowerMechanism();
        flywheelSpeedMode = FlywheelSpeedMode.AUTO;
        flywheelsEnabled = false;
        manualTargetVelocity = 0.0;
        previousY = previousSpeedModeButton = previousDpadUp = previousDpadDown = false;
        telemetry.addData("Status", "Drivetrain, intake, transfer, and flywheels initialized");
        telemetry.addData("Speed scale", driveTrain.getSpeedScale());
        telemetry.addData("Flywheel speed status", "AUTO UNCALIBRATED");
        addBallSelectionTelemetry();
        addFlowerTelemetry();
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
        // Gamepad 1 right bumper is reserved for future hold-to-aim; it has no effect here.
        driveTrain.drive(forward, strafe, turn);

        // Hold-to-run; reverse takes priority independently for each mechanism.
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            intake.reverse();
        } else if (gamepad2.a || gamepad2.x) {
            intake.intake();
        } else {
            intake.stop();
        }

        // Manual transfer can feed directly into the shooter; there is no readiness interlock.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            transfer.reverse();
        } else if (gamepad2.b || gamepad2.x) {
            transfer.feed();
        } else {
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
                ? "AUTO UNCALIBRATED" : (isManualFlywheelConfigured() ? "MANUAL" : "MANUAL UNCONFIGURED"));
        telemetry.addData("Manual target (ticks/s)", manualTargetVelocity);
        telemetry.addData("Mode target (ticks/s)", flywheelTargetVelocity);
        telemetry.addData("Commanded velocity (ticks/s)", shooter.getTargetVelocity());
        telemetry.addData("Flywheel A (ticks/s)", shooter.getVelocityA());
        telemetry.addData("Flywheel B (ticks/s)", shooter.getVelocityB());
        telemetry.addData("Flywheels at speed", flywheelsAtSpeed);
        addBallSelectionTelemetry();
        addFlowerTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (driveTrain != null) {
            driveTrain.stop();
        }
        if (intake != null) {
            intake.stop();
        }
        if (transfer != null) {
            transfer.stop();
        }
        if (shooter != null) {
            shooter.stop();
        }
        flywheelsEnabled = false;
    }

    private void initializeFlowerMechanism() {
        flowerMechanism = hardwareMap.tryGet(Servo.class, "flowerServo") != null
                ? new FlowerMechanism(hardwareMap) : null;
        flowerPositionsConfigured = false;
        if (flowerMechanism != null && isServoPositionConfigured(FLOWER_RAISED_POSITION)
                && isServoPositionConfigured(FLOWER_LOWERED_POSITION)) {
            // Configures presets only; the subsystem does not command a position here.
            flowerMechanism.configurePositions(FLOWER_RAISED_POSITION, FLOWER_LOWERED_POSITION);
            flowerPositionsConfigured = true;
        }
        previousDpadLeft = previousDpadRight = false;
    }

    private void updateFlowerMechanism() {
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        if (flowerMechanism != null && flowerPositionsConfigured) {
            if (left && !previousDpadLeft && !right) {
                flowerMechanism.lower();
            } else if (right && !previousDpadRight && !left) {
                flowerMechanism.raise();
            }
        }
        // Consume both edges during a conflict; releasing one does not command the held one.
        previousDpadLeft = left;
        previousDpadRight = right;
    }

    private void addFlowerTelemetry() {
        telemetry.addData("Flower servo available", flowerMechanism != null);
        telemetry.addData("Flower position calibration", flowerPositionsConfigured ? "CONFIGURED" : "UNCALIBRATED");
        telemetry.addData("Flower commanded state", flowerMechanism == null ? "UNAVAILABLE" : flowerMechanism.getState());
        double target = flowerMechanism == null ? Double.NaN : flowerMechanism.getTargetPosition();
        // Last commanded target only; the positional servo provides no physical position feedback.
        telemetry.addData("Flower commanded target", Double.isNaN(target) ? "UNCOMMANDED" : target);
    }

    private void initializeBallSelection() {
        adjustmentAvailable = hardwareMap.tryGet(Servo.class, "shooterAdjust") != null;
        adjustmentCalibrated = isServoPositionConfigured(POLLEN_ADJUSTMENT_POSITION)
                && isServoPositionConfigured(NECTAR_ADJUSTMENT_POSITION)
                && Double.isFinite(ADJUSTMENT_SETTLING_SECONDS) && ADJUSTMENT_SETTLING_SECONDS >= 0.0;
        // Choose exactly one Shooter before creating any flywheel motor objects.
        // Both motor directions remain FORWARD, as in the previous flywheel-only constructor.
        if (adjustmentAvailable && adjustmentCalibrated) {
            shooter = new Shooter(hardwareMap, DcMotorSimple.Direction.FORWARD,
                    DcMotorSimple.Direction.FORWARD, FLYWHEEL_VELOCITY_TOLERANCE,
                    FLYWHEEL_STABLE_TIME_SECONDS, ADJUSTMENT_DIRECTION, ADJUSTMENT_SETTLING_SECONDS);
            shooter.configureAdjustmentPositions(POLLEN_ADJUSTMENT_POSITION, NECTAR_ADJUSTMENT_POSITION);
        } else {
            shooter = new Shooter(hardwareMap, FLYWHEEL_VELOCITY_TOLERANCE,
                    FLYWHEEL_STABLE_TIME_SECONDS);
        }
        ballSelectionMode = Shooter.SelectionMode.AUTO;
        previousBallSelectionButton = false;
        shooter.setAutoMode(); // No position command, even with a fully calibrated servo.

        colorSensorAvailable = hardwareMap.tryGet(ColorSensor.class, "ballColor") != null;
        // The sensor API requires positive stability time; leave AUTO unavailable until configured.
        ballColorSensor = Double.isFinite(COLOR_STABLE_TIME_SECONDS) && COLOR_STABLE_TIME_SECONDS > 0.0
                ? new BallColorSensor(hardwareMap, COLOR_STABLE_TIME_SECONDS) : null;
        if (ballColorSensor != null && POLLEN_COLOR_PROFILE != null && NECTAR_COLOR_PROFILE != null
                && COLOR_MINIMUM_SIGNAL >= 0) {
            ballColorSensor.configureCalibration(POLLEN_COLOR_PROFILE, NECTAR_COLOR_PROFILE,
                    COLOR_MINIMUM_SIGNAL);
        }
        shootingCoordinator = new ShootingCoordinator(shooter, ballColorSensor);
    }

    private void updateBallSelectionMode() {
        boolean button = gamepad2.right_bumper;
        if (button && !previousBallSelectionButton) {
            switch (ballSelectionMode) {
                case AUTO:
                    ballSelectionMode = Shooter.SelectionMode.MANUAL_POLLEN;
                    break;
                case MANUAL_POLLEN:
                    ballSelectionMode = Shooter.SelectionMode.MANUAL_NECTAR;
                    break;
                default:
                    ballSelectionMode = Shooter.SelectionMode.AUTO;
                    break;
            }
            if (ballSelectionMode == Shooter.SelectionMode.AUTO || (adjustmentAvailable && adjustmentCalibrated)) {
                shooter.setSelectionMode(ballSelectionMode);
            }
        }
        previousBallSelectionButton = button;
    }

    private void addBallSelectionTelemetry() {
        telemetry.addData("Ball-selection mode", ballSelectionMode);
        telemetry.addData("Detected ball type", shootingCoordinator.getDetectedBallType());
        telemetry.addData("Requested ball type", getRequestedBallType());
        telemetry.addData("Applied ball type", shooter.getSelectedBallType());
        telemetry.addData("Color sensor mapped", colorSensorAvailable);
        telemetry.addData("Color calibration", ballColorSensor != null && ballColorSensor.isCalibrated()
                ? "CONFIGURED" : "UNCALIBRATED");
        telemetry.addData("Color sensor status", ballColorSensor != null ? ballColorSensor.getStatus()
                : (colorSensorAvailable ? "UNCALIBRATED STABILITY TIME" : "MISSING"));
        telemetry.addData("Shooter servo mapped", adjustmentAvailable);
        telemetry.addData("Shooter servo calibration", adjustmentCalibrated ? "CONFIGURED" : "UNCALIBRATED");
        double position = shooter.getAdjustmentTargetPosition();
        telemetry.addData("Shooter servo command", Double.isNaN(position) ? "UNCOMMANDED" : position);
        telemetry.addData("Shooter servo settled", shooter.isAdjustmentSettled());
        // Software readiness only; does not confirm safe staging or operate the manual transfer.
        telemetry.addData("Shooter ready to feed", shootingCoordinator.isReadyToFeed());
    }

    /** Driver intent for telemetry, not confirmation that a servo command could be applied. */
    private BallColorSensor.BallType getRequestedBallType() {
        if (ballSelectionMode == Shooter.SelectionMode.MANUAL_POLLEN) {
            return BallColorSensor.BallType.POLLEN;
        }
        if (ballSelectionMode == Shooter.SelectionMode.MANUAL_NECTAR) {
            return BallColorSensor.BallType.NECTAR;
        }
        return shootingCoordinator.getDetectedBallType();
    }

    private static boolean isServoPositionConfigured(double position) {
        return Double.isFinite(position) && position >= 0.0 && position <= 1.0;
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

        boolean manualConfigured = isManualFlywheelConfigured();
        if (flywheelSpeedMode == FlywheelSpeedMode.MANUAL && manualConfigured) {
            // Ignore conflicting D-pad directions; adjustments are allowed while OFF.
            if (up && !previousDpadUp && !down) {
                manualTargetVelocity = Math.min(MANUAL_FLYWHEEL_MAX_VELOCITY,
                        manualTargetVelocity + MANUAL_FLYWHEEL_INCREMENT);
            } else if (down && !previousDpadDown && !up) {
                manualTargetVelocity = Math.max(0.0,
                        manualTargetVelocity - MANUAL_FLYWHEEL_INCREMENT);
            }
        }
        // Track edges even in AUTO or while unconfigured; a held button is not a new press.
        previousY = y;
        previousSpeedModeButton = speedModeButton;
        previousDpadUp = up;
        previousDpadDown = down;

        // AUTO has no calibrated target yet and must never reuse the manual speed.
        double targetVelocity = flywheelSpeedMode == FlywheelSpeedMode.MANUAL && manualConfigured
                ? manualTargetVelocity : 0.0;
        shooter.setTargetVelocity(flywheelsEnabled ? targetVelocity : 0.0);
        return targetVelocity;
    }

    private static boolean isManualFlywheelConfigured() {
        return Double.isFinite(MANUAL_FLYWHEEL_MAX_VELOCITY) && MANUAL_FLYWHEEL_MAX_VELOCITY > 0.0
                && Double.isFinite(MANUAL_FLYWHEEL_INCREMENT) && MANUAL_FLYWHEEL_INCREMENT > 0.0
                && MANUAL_FLYWHEEL_INCREMENT <= MANUAL_FLYWHEEL_MAX_VELOCITY;
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) <= JOYSTICK_DEADZONE ? 0.0 : value;
    }
}
