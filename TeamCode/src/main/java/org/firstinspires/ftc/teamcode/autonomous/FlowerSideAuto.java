package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * SECOND autonomous starting on the flower side.
 * Independent from RedAuto / RedAutoPlan. Route offsets are ROBOT-RELATIVE.
 * This OpMode intentionally refuses to start until all unmeasured offsets are filled in.
 *
 * Routine:
 *   clear wall -> shoot preload -> RIGHT 180 -> lower flower mechanism ->
 *   forward + LEFT while intaking -> brief collection hold -> reverse out ->
 *   raise flower mechanism -> turn back toward hive -> shoot collected balls -> park.
 *
 * No Limelight or ball-colour auto switching in this version: each volley uses an
 * explicitly configured ball type and tested target velocity. Do not use mixed
 * Pollen/Nectar in a single volley without adding per-ball identification/feeding.
 */
@Autonomous(name = "BIOBUZZ - Flower Side Auto", group = "Competition")
public class FlowerSideAuto extends OpMode {

    // ======================== MEASURE THESE ON THE FIELD ========================
    // All distances are in inches in the robot's LOCAL coordinates at that step.
    // +forward = toward robot front; +left = toward robot left.
    // Do NOT guess these numbers. Start will be blocked while any remains NaN.
    private static final double START_FORWARD_IN = 10.0;
    private static final double FLOWER_FORWARD_IN = 3.0;
    private static final double FLOWER_LEFT_IN = 5.0;
    private static final double PARK_FORWARD_IN = 5.0;
    private static final double PARK_LEFT_IN = -15.0;

    // Right turn is negative in the coordinate convention used by RedAutoPlan.
    private static final double FIRST_TURN_DEG = -180.0;
    // 180 degrees to face the hive again. Change if the actual hive angle differs.
    private static final double RETURN_TURN_DEG = -180.0;

    // ======================== SHOOTER CALIBRATION ==============================
    // These speeds are EXAMPLES for a ~65-inch shot, not verified on the new side.
    // Configure BOTH the ball type and speed for each shooting position.
    private static final boolean PRELOAD_IS_NECTAR = false;
    private static final double PRELOAD_SPEED_TPS = 1150.0;
    private static final boolean COLLECTED_IS_NECTAR = false;
    private static final double COLLECTED_SPEED_TPS = 1150.0;

    private static final double POLLEN_SERVO_POS = 0.10;
    private static final double NECTAR_SERVO_POS = 0.00;
    private static final double ADJUSTMENT_SETTLE_S = 0.05; // Verify actual ramp travel time.
    private static final double FLYWHEEL_TOLERANCE_TPS = 75.0;
    private static final double FLYWHEEL_STABLE_S = 0.25;

    private static final double FLOWER_RAISED_POS = 0.0;
    private static final double FLOWER_LOWERED_POS = 0.3;

    // ======================== STEP TIMING / MOTORS ============================
    private static final double PRELOAD_FEED_S = 2.5;
    private static final double COLLECTED_FEED_S = 2.5;
    private static final double COLLECT_HOLD_S = 1.25;
    private static final double FLOWER_SERVO_WAIT_S = 0.45;
    private static final double SHOOT_STEP_TIMEOUT_S = 5.0;
    private static final double DRIVE_STEP_TIMEOUT_S = 7.0;
    private static final double TURN_STEP_TIMEOUT_S = 4.5;
    private static final double AUTO_LIMIT_S = 29.0;

    private static final double PICKUP_INTAKE_POWER = 0.60;
    private static final double PICKUP_TRANSFER_POWER = 0.15; // Required to prevent jams.
    private static final double SHOOT_INTAKE_POWER = 1.00;
    private static final double SHOOT_TRANSFER_POWER = 1.00;

    private static final double DRIVE_TOLERANCE_IN = 1.0;
    private static final double TURN_TOLERANCE_DEG = 5.0;
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(5.0);
    private static final double PEDRO_ENDPOINT_TIMEOUT_MS = 11000.0;

    private enum Step {
        CLEAR_FORWARD,
        SHOOT_PRELOAD,
        LOWER_FLOWER,
        TURN_RIGHT_180,

        APPROACH_FLOWER,
        COLLECT_FLOWER,
        RETREAT_FLOWER,
        RAISE_FLOWER,
        TURN_TO_HIVE,
        SHOOT_COLLECTED,
        PARK
    }

    private DriveTrain driveTrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private FlowerMechanism flower;
    private Follower follower;

    private Step step;
    private Pose plannedPose;
    private Pose target;
    private Path activePath;
    private long autoStartedNs;
    private long stepStartedNs;
    private long lastFeedSampleNs;
    private double fedSeconds;
    private int skippedVolleys;
    private boolean running;
    private String status = "INITIALIZING";
    private String feedStatus = "IDLE";
    private String configError;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap, DcMotorSimple.Direction.REVERSE, 1.0);
        transfer = new Transfer(hardwareMap, DcMotorSimple.Direction.REVERSE, 1.0);
        shooter = new Shooter(hardwareMap,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                FLYWHEEL_TOLERANCE_TPS, FLYWHEEL_STABLE_S,
                Servo.Direction.FORWARD, ADJUSTMENT_SETTLE_S);
        shooter.configureAdjustmentPositions(POLLEN_SERVO_POS, NECTAR_SERVO_POS);
        flower = new FlowerMechanism(hardwareMap,
                Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        flower.configurePositions(FLOWER_RAISED_POS, FLOWER_LOWERED_POS);

        follower = Constants.createFollower(hardwareMap);
        plannedPose = new Pose(0, 0, 0);
        follower.setStartingPose(plannedPose);

        configError = validateRoute();
        status = configError == null ? "READY" : "NOT READY - EDIT DISTANCES";
        showTelemetry();
    }

    @Override
    public void init_loop() {
        showTelemetry();
    }

    @Override
    public void start() {
        if (configError != null || gamepad1.b || gamepad2.b) {
            status = configError == null ? "CANCELLED" : "BLOCKED - ROUTE UNMEASURED";
            shutdown();
            requestOpModeStop();
            return;
        }

        plannedPose = new Pose(0, 0, 0);
        follower.setStartingPose(plannedPose);
        setShot(PRELOAD_IS_NECTAR, PRELOAD_SPEED_TPS);
        flower.raise();

        running = true;
        autoStartedNs = System.nanoTime();
        enter(Step.CLEAR_FORWARD, autoStartedNs);
    }

    @Override
    public void loop() {
        if (!running) {
            showTelemetry();
            return;
        }

        if (gamepad1.b || gamepad2.b) {
            fail("CANCELLED WITH B");
            return;
        }

        long now = System.nanoTime();
        if (elapsed(now, autoStartedNs) >= AUTO_LIMIT_S) {
            fail("AUTO TIME LIMIT REACHED");
            return;
        }

        follower.update();

        switch (step) {
            case CLEAR_FORWARD:
            case APPROACH_FLOWER:
            case RETREAT_FLOWER:
            case PARK:
                checkDrive(now);
                break;

            case TURN_RIGHT_180:
            case TURN_TO_HIVE:
                checkTurn(now);
                break;

            case LOWER_FLOWER:
            case RAISE_FLOWER:
                if (elapsed(now, stepStartedNs) >= FLOWER_SERVO_WAIT_S) {
                    advance(now);
                }
                break;

            case COLLECT_FLOWER:
                if (elapsed(now, stepStartedNs) >= COLLECT_HOLD_S) {
                    advance(now);
                }
                break;

            case SHOOT_PRELOAD:
            case SHOOT_COLLECTED:
                updateShooting(now);
                break;
        }
        showTelemetry();
    }

    private void enter(Step next, long now) {
        step = next;
        stepStartedNs = now;
        activePath = null;
        fedSeconds = 0.0;
        lastFeedSampleNs = now;
        target = plannedPose;
        feedStatus = "IDLE";
        status = "RUNNING";

        // Stop staging motors at stage boundaries; exceptions below turn them on.
        intake.stop();
        transfer.stop();

        switch (next) {
            case CLEAR_FORWARD:
                startDrive(START_FORWARD_IN, 0.0, 0.35);
                break;
            case SHOOT_PRELOAD:
                feedStatus = "WAITING FOR READINESS";
                break;
            case TURN_RIGHT_180:
                startTurn(FIRST_TURN_DEG, 0.60);
                break;
            case LOWER_FLOWER:
                flower.lower();
                break;
            case APPROACH_FLOWER:
                runPickup();
                startDrive(FLOWER_FORWARD_IN, FLOWER_LEFT_IN, 0.50);
                break;
            case COLLECT_FLOWER:
                runPickup();
                break;
            case RETREAT_FLOWER:
                // Change the flywheel target early while driving out of the flower.
                setShot(COLLECTED_IS_NECTAR, COLLECTED_SPEED_TPS);
                runPickup(); // 15% transfer prevents a partly collected ball jamming.
                startDrive(-FLOWER_FORWARD_IN, -FLOWER_LEFT_IN, 0.50);
                break;
            case RAISE_FLOWER:
                flower.raise();
                break;
            case TURN_TO_HIVE:
                startTurn(RETURN_TURN_DEG, 0.60);
                break;
            case SHOOT_COLLECTED:
                feedStatus = "WAITING FOR READINESS";
                break;
            case PARK:
                startDrive(PARK_FORWARD_IN, PARK_LEFT_IN, 0.65);
                break;
        }
    }

    private void startDrive(double forward, double left, double power) {
        Pose from = follower.getPose();
        plannedPose = translate(plannedPose, forward, left);
        target = plannedPose;
        activePath = new Path(new BezierLine(from, target), new PathConstraints(
                0.995, 1.0, DRIVE_TOLERANCE_IN,
                HEADING_TOLERANCE_RAD, PEDRO_ENDPOINT_TIMEOUT_MS,
                Constants.pathConstraints.getBrakingStrength(),
                Constants.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(),
                Constants.pathConstraints.getBrakingStart()));
        activePath.setConstantHeadingInterpolation(target.getHeading());
        follower.setMaxPower(Math.min(power, Constants.mecanumMaxPower));
        follower.followPath(activePath, false);
    }

    private void startTurn(double turnDegrees, double power) {
        double delta = Math.toRadians(turnDegrees);
        plannedPose = new Pose(plannedPose.getX(), plannedPose.getY(),
                normalize(plannedPose.getHeading() + delta));
        target = plannedPose;
        follower.setMaxPower(Math.min(power, Constants.mecanumMaxPower));
        // Preserve the requested CLOCKWISE direction even for exactly 180 degrees.
        follower.turn(delta);
    }

    private void checkDrive(long now) {
        Pose actual = follower.getPose();
        double distance = Math.hypot(target.getX() - actual.getX(),
                target.getY() - actual.getY());
        if (distance <= DRIVE_TOLERANCE_IN) {
            follower.breakFollowing();
            driveTrain.stop();
            advance(now);
        } else if (elapsed(now, stepStartedNs) >= DRIVE_STEP_TIMEOUT_S) {
            fail("DRIVE TIMEOUT: " + step);
        }
    }

    private void checkTurn(long now) {
        double headingError = Math.atan2(
                Math.sin(target.getHeading() - follower.getPose().getHeading()),
                Math.cos(target.getHeading() - follower.getPose().getHeading()));
        if (Math.abs(headingError) <= Math.toRadians(TURN_TOLERANCE_DEG)) {
            follower.breakFollowing();
            driveTrain.stop();
            advance(now);
        } else if (elapsed(now, stepStartedNs) >= TURN_STEP_TIMEOUT_S) {
            fail("TURN TIMEOUT: " + step);
        }
    }

    private void updateShooting(long now) {
        // Re-check every loop. If a flywheel slows down, stop the feeder immediately.
        boolean ready = shooter.isReadyToFeed();
        if (ready) {
            intake.setPower(SHOOT_INTAKE_POWER);
            transfer.setPower(SHOOT_TRANSFER_POWER);
            fedSeconds += elapsed(now, lastFeedSampleNs);
            feedStatus = String.format("FEEDING %.2f s", fedSeconds);
        } else {
            intake.stop();
            transfer.stop();
            feedStatus = "WAITING FOR SPEED + COMPRESSION";
        }
        lastFeedSampleNs = now;

        double required = step == Step.SHOOT_PRELOAD
                ? PRELOAD_FEED_S : COLLECTED_FEED_S;
        if (fedSeconds >= required) {
            intake.stop();
            transfer.stop();
            advance(now);
        } else if (elapsed(now, stepStartedNs) >= SHOOT_STEP_TIMEOUT_S) {
            skippedVolleys++;
            intake.stop();
            transfer.stop();
            advance(now);
        }
    }

    private void runPickup() {
        intake.setPower(PICKUP_INTAKE_POWER);
        transfer.setPower(PICKUP_TRANSFER_POWER);
    }

    private void setShot(boolean nectar, double ticksPerSecond) {
        if (nectar) {
            shooter.setNectarMode();
        } else {
            shooter.setPollenMode();
        }
        // Flywheels stay running between shots and during all drives/turns.
        shooter.setTargetVelocity(ticksPerSecond);
    }

    private void advance(long now) {
        if (step == Step.PARK) {
            status = "COMPLETE - PARKED";
            shutdown();
            requestOpModeStop();
            return;
        }
        Step[] steps = Step.values();
        enter(steps[step.ordinal() + 1], now);
    }

    private void fail(String reason) {
        status = "FAILED - " + reason;
        shutdown();
        requestOpModeStop();
    }

    private void shutdown() {
        running = false;
        stopDevice("Pedro", () -> { if (follower != null) follower.breakFollowing(); });
        stopDevice("drivetrain", () -> { if (driveTrain != null) driveTrain.stop(); });
        stopDevice("intake", () -> { if (intake != null) intake.stop(); });
        stopDevice("transfer", () -> { if (transfer != null) transfer.stop(); });
        stopDevice("flywheels", () -> { if (shooter != null) shooter.stop(); });
    }

    @Override
    public void stop() {
        shutdown();
    }

    private void stopDevice(String name, Runnable stop) {
        try { stop.run(); }
        catch (RuntimeException ex) {
            RobotLog.ee("FlowerSideAuto", ex, "Failed to stop " + name);
        }
    }

    /** Reject unmeasured positions instead of unexpectedly driving toward a field element. */
    private String validateRoute() {
        if (!Double.isFinite(START_FORWARD_IN) || START_FORWARD_IN <= 0.0) {
            return "Set START_FORWARD_IN > 0";
        }
        if (!Double.isFinite(FLOWER_FORWARD_IN) || FLOWER_FORWARD_IN <= 0.0) {
            return "Set FLOWER_FORWARD_IN > 0";
        }
        if (!Double.isFinite(FLOWER_LEFT_IN) || FLOWER_LEFT_IN <= 0.0) {
            return "Set FLOWER_LEFT_IN > 0";
        }
        if (!Double.isFinite(PARK_FORWARD_IN) || !Double.isFinite(PARK_LEFT_IN)
                || Math.hypot(PARK_FORWARD_IN, PARK_LEFT_IN) < 0.01) {
            return "Set PARK_FORWARD_IN / PARK_LEFT_IN";
        }
        if (!(PRELOAD_SPEED_TPS > 0.0 && COLLECTED_SPEED_TPS > 0.0)) {
            return "Set valid shooting speeds";
        }
        return null;
    }

    private void showTelemetry() {
        telemetry.addData("Flower-side Auto", status);
        telemetry.addData("Setup error", configError == null ? "NONE" : configError);
        telemetry.addData("Step", step == null ? "NOT STARTED" : step);
        if (follower != null) {
            Pose pose = follower.getPose();
            telemetry.addData("Pedro pose", String.format("%.1f, %.1f, %.0f deg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        }
        telemetry.addData("Flywheel target", shooter == null ? 0.0 : shooter.getTargetVelocity());
        telemetry.addData("Shooter ready", shooter != null && shooter.isReadyToFeed());
        telemetry.addData("Feed", feedStatus);
        telemetry.addData("Skipped volley count", skippedVolleys);
        telemetry.addLine("B (either gamepad) = emergency cancel.");
        telemetry.update();
    }

    private static double elapsed(long now, long start) {
        return (now - start) / 1_000_000_000.0;
    }

    private static Pose translate(Pose from, double forward, double left) {
        double heading = from.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Pose(from.getX() + forward * cos - left * sin,
                from.getY() + forward * sin + left * cos, heading);
    }

    private static double normalize(double radians) {
        return (radians % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }
}
