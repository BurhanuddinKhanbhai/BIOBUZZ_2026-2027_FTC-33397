package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.SafeThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

/**
 * Exact sixteen-step route. All movement uses Pedro; time only bounds actions and feed attempts.
 * No ball-exit sensor exists: a finished pulse is an ATTEMPT, never a counted successful shot.
 * This OpMode remains inert until BOTH its preflight and Constants' existing review guard pass.
 */
@Autonomous(name = "BIOBUZZ - Red Auto", group = "Competition")
public class RedAuto extends OpMode {
    /** Edit measured values here; these are not live dashboard settings during a run. */
    public static final class Settings {
        // Must include clearance from game structures, wall-turn sweep and feed testing.
        public boolean routeAndMechanismsVerified = false;
        public double startX = Double.NaN, startY = Double.NaN; // Tracking origin; heading PI (left).
        public double fieldWidth = Double.NaN, fieldHeight = Double.NaN; // Verified inside dimensions.
        public double sweptRobotRadius = Double.NaN; // Includes both flower arms when lowered.
        public double boundaryMargin = Double.NaN; // Tracking error plus stopping/coasting clearance.
        public double intake2Distance = Double.NaN;

        // Conservative software limits, not measured follower tuning or a guarantee of stopping distance.
        public double pathPower = 0.25, positionTolerance = 1.0;
        public double headingTolerance = Math.toRadians(3.0);
        public double stoppedSpeed = 1.0, stoppedAngularSpeed = 0.10;
        public double pathTimeout = 10.0, turnTimeout = 6.0, noProgressTimeout = 2.0;
        // Longer than our whole-path watchdog: Pedro's endpoint timeout can NEVER mean success.
        public double endpointTimeoutMs = 11_000.0, maxLoopGap = 0.25;
        public double autonomousDeadline = 29.0; // Safety budget; verify against event period.
        public double intakePower = 0.35, transferPower = 0.35;
        public double flywheelTolerance = 75.0, flywheelStableSeconds = 0.25;
        public final FeedSettings shoot1 = new FeedSettings(4);
        public final FeedSettings shoot2 = new FeedSettings(4);
        public final FeedSettings shoot3 = new FeedSettings(0); // Number collected is not yet specified.

        // Preserve current local competition presets/direction. No positions commanded in init.
        public double pollenPosition = 0.21, nectarPosition = 0.00, adjustmentSettlingSeconds = 0.05;
        public double flower1Raised = 0.31, flower1Lowered = 0.83;
        // ServoCalibration has only live test targets, NOT calibrated endpoints for servo 2.
        public double flower2Lowered = Double.NaN, flowerSettlingSeconds = Double.NaN;
        public Servo.Direction flower2Direction = null;
        public double flowerTimeout = 5.0;
    }

    public static final class FeedSettings {
        public int attempts;
        public double onSeconds = Double.NaN, gapSeconds = Double.NaN;
        public double timeoutSeconds = 8.0;
        public FeedSettings(int attempts) { this.attempts = attempts; }
    }

    private static final String[] MOTOR_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight",
            "intake", "transfer", "flywheelA", "flywheelB"};
    private static final double SHOOTING_TICKS_PER_SECOND = 1500.0;
    private final Settings settings;
    private final LongSupplier clock;
    private final DcMotorEx[] motors = new DcMotorEx[MOTOR_NAMES.length];
    private RedAutoPlan plan;
    private RedAutoPlan.Step step;
    private Follower follower;
    private SafeThreeWheelLocalizer localizer;
    private DriveTrain driveTrain; // Initialization and independent zero-output fallback ONLY.
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private FlowerMechanism flower;
    private Servo flower2;
    private BallColorSensor color;
    private Path activePath;
    private Pose target, pose, progressPose;
    private boolean prepared, running, started, poseGuardArmed, feeding, shutdownDone, motionEnded;
    private boolean atSpeed, feedReady;
    private int attemptsCompleted;
    private long autoStarted, stepStarted, lastLoop, lastProgress, feedStarted, gapStarted;
    private double velocityA = Double.NaN, velocityB = Double.NaN;
    private double speed = Double.NaN, angularSpeed = Double.NaN;
    private String status = "NOT INITIALIZED", error = "", feedStatus = "STOPPED";
    private String flowerStatus = "UNCOMMANDED; calibration pending";
    private BallColorSensor.BallType attemptBall = BallColorSensor.BallType.UNKNOWN;

    public RedAuto() { this(new Settings(), System::nanoTime); }

    // Deterministic clock/config seam for host tests; the SDK uses the public no-arg constructor.
    RedAuto(Settings settings, LongSupplier clock) { this.settings = settings; this.clock = clock; }

    @Override public void init() {
        List<String> issues = configurationIssues();
        String pedroIssue = Constants.getConfigurationIssue();
        if (pedroIssue != null) issues.add(pedroIssue);
        try {
            plan = new RedAutoPlan(settings.startX, settings.startY, settings.intake2Distance,
                    settings.fieldWidth, settings.fieldHeight, settings.sweptRobotRadius, settings.boundaryMargin);
        } catch (IllegalArgumentException e) { issues.add(e.getMessage()); }
        try {
            // Lookups and calibration loading do not command positions or read encoder channels.
            for (int i = 0; i < motors.length; i++) {
                motors[i] = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAMES[i]);
                if (motors[i] == null) issues.add("Missing motor: " + MOTOR_NAMES[i]);
            }
            for (String name : new String[]{"shooterAdjust", "flowerServo", "flowerServo2"}) {
                if (hardwareMap.tryGet(Servo.class, name) == null) issues.add("Missing servo: " + name);
            }
            color = new BallColorSensor(hardwareMap);
            if (!color.isAvailable()) issues.add("Missing ballColor sensor");
            if (!color.isCalibrated()) issues.add("Color calibration: " + color.getCalibrationStatus());
            if (!color.isRedCalibrated()) issues.add("Red nectar calibration missing");
        } catch (RuntimeException e) { issues.add("Hardware lookup/calibration: " + e.getMessage()); }
        // Even a blocked INIT must clear residual motor output. These independent zero writes
        // never reset encoders, change modes, move servos, or enable the guarded follower.
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] == null) continue;
            try { motors[i].setPower(0.0); }
            catch (RuntimeException e) {
                issues.add("Initial stop failed: " + MOTOR_NAMES[i]);
                RobotLog.ee("RedAuto", e, "Initial stop failed: " + MOTOR_NAMES[i]);
            }
        }
        if (!issues.isEmpty()) {
            status = "BLOCKED - no actions enabled";
            error = String.join("; ", issues);
            showTelemetry();
            return;
        }
        try {
            validateWiring();
            // Match CURRENT TeleOp directions, before localizer captures any encoder baseline.
            driveTrain = new DriveTrain(hardwareMap);
            intake = new Intake(hardwareMap, DcMotorSimple.Direction.REVERSE, settings.intakePower);
            transfer = new Transfer(hardwareMap, DcMotorSimple.Direction.REVERSE, settings.transferPower);
            shooter = new Shooter(hardwareMap, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                    settings.flywheelTolerance, settings.flywheelStableSeconds,
                    Servo.Direction.FORWARD, settings.adjustmentSettlingSeconds);
            shooter.configureAdjustmentPositions(settings.pollenPosition, settings.nectarPosition);
            shooter.setAutoMode();
            flower = new FlowerMechanism(hardwareMap);
            flower.configurePositions(settings.flower1Raised, settings.flower1Lowered);
            flower2 = hardwareMap.get(Servo.class, "flowerServo2");
            flower2.setDirection(settings.flower2Direction);
            flowerStatus = "Both presets configured; UNCOMMANDED";
            follower = Constants.createFollower(hardwareMap, next -> {
                // Factory invokes this AFTER localization, BEFORE Pedro writes drivetrain output.
                if (poseGuardArmed && !plan.contains(next)) {
                    throw new IllegalStateException("Localization outside verified field/robot clearance");
                }
            });
            localizer = (SafeThreeWheelLocalizer) follower.getPoseTracker().getLocalizer();
            follower.setStartingPose(plan.getStart()); // Software baseline only, never encoder reset.
            pose = plan.getStart();
            prepared = true;
            status = "READY - route preflight passed";
        } catch (RuntimeException e) { fail("Initialization failed", e); }
        showTelemetry();
    }

    @Override public void init_loop() { showTelemetry(); }

    @Override public void start() {
        if (started || !prepared || shutdownDone) return;
        started = true;
        try {
            require(!gamepad1.b && !gamepad2.b, "Operator emergency stop held at START (B)");
            // Rebaseline time after the INIT wait. Robot must still be at the configured start mark.
            follower.setStartingPose(plan.getStart());
            poseGuardArmed = true;
            autoStarted = lastLoop = clock.getAsLong();
            running = true;
            enter(RedAutoPlan.Step.SHOOT_1, autoStarted);
        } catch (RuntimeException e) { fail("Start failed", e); }
    }

    @Override public void loop() {
        if (running) {
            try {
                long now = clock.getAsLong();
                require(!gamepad1.b && !gamepad2.b, "Operator emergency stop (B)");
                require(elapsed(now, autoStarted) < settings.autonomousDeadline, "Autonomous deadline expired");
                require(now >= lastLoop && elapsed(now, lastLoop) <= settings.maxLoopGap, "Control-loop gap exceeded safety limit");
                // Check action deadlines before Pedro gets another opportunity to command output.
                double actionLimit = RedAutoPlan.isDrive(step) ? settings.pathTimeout
                        : RedAutoPlan.isTurn(step) ? settings.turnTimeout
                        : RedAutoPlan.isShoot(step) ? feedSettings().timeoutSeconds : settings.flowerTimeout;
                require(elapsed(now, stepStarted) < actionLimit, "Action timeout at " + step + "; NOT completed");
                lastLoop = now;
                follower.update(); // Exactly once; guarded updatePose prevents output on invalid feedback.
                pose = follower.getPose();
                speed = follower.getVelocity().getMagnitude();
                angularSpeed = follower.getAngularVelocity();
                require(localizer.isSampleValid() && !follower.isLocalizationNAN()
                                && Double.isFinite(speed) && Double.isFinite(angularSpeed),
                        "Invalid localization/velocity: " + localizer.getStatus());
                color.update(); // Exactly one sensor update; no second coordinator read.
                require(color.hasColorSample(), "Color sensor unavailable/invalid: " + color.getStatus());
                velocityA = shooter.getVelocityA();
                velocityB = shooter.getVelocityB();
                require(Double.isFinite(velocityA) && Double.isFinite(velocityB), "Invalid flywheel velocity feedback");
                atSpeed = shooter.isAtSpeed();
                if (RedAutoPlan.isDrive(step) || RedAutoPlan.isTurn(step)) updateMotion(now);
                else if (RedAutoPlan.isShoot(step)) updateShooting(now);
                else updateFlower(now);
            } catch (RuntimeException e) { fail("Autonomous stopped", e); }
        }
        showTelemetry();
    }

    private void enter(RedAutoPlan.Step next, long now) {
        step = next;
        target = plan.getTarget(step);
        stepStarted = lastProgress = now;
        progressPose = follower.getPose();
        activePath = null;
        motionEnded = false;
        status = "RUNNING";
        // Intake operates only on the two named collection paths. Return paths never reverse it.
        if (step == RedAutoPlan.Step.INTAKE_1 || step == RedAutoPlan.Step.INTAKE_2) intake.intake();
        else intake.stop();
        transfer.stop();
        feeding = false;
        feedReady = false;
        atSpeed = false;
        if (RedAutoPlan.isShoot(step)) {
            attemptsCompleted = 0;
            gapStarted = now;
            shooter.setAutoMode();
            shooter.setTargetVelocity(SHOOTING_TICKS_PER_SECOND);
            feedStatus = "WAITING FOR SPEED / BALL / SERVO";
        } else {
            shooter.stop();
            feedStatus = "STOPPED";
        }
        if (RedAutoPlan.isDrive(step)) {
            Pose start = step.ordinal() == 0 ? plan.getStart() : plan.getTarget(RedAutoPlan.Step.values()[step.ordinal() - 1]);
            activePath = new Path(new BezierLine(start, target), new PathConstraints(
                    0.995, settings.stoppedSpeed, settings.positionTolerance, settings.headingTolerance,
                    settings.endpointTimeoutMs, Constants.pathConstraints.getBrakingStrength(),
                    Constants.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(), Constants.pathConstraints.getBrakingStart()));
            activePath.setConstantHeadingInterpolation(target.getHeading());
            follower.setMaxPower(Math.min(settings.pathPower, Constants.mecanumMaxPower));
            follower.followPath(activePath, false);
        } else if (RedAutoPlan.isTurn(step)) {
            follower.setMaxPower(Math.min(settings.pathPower, Constants.mecanumMaxPower));
            // Correct to the PLANNED cardinal heading; don't accumulate prior heading error.
            // Pedro chooses the shortest heading error; a half-turn may rotate either way.
            double rotation = RedAutoPlan.turnRadians(step) > 0.0
                    ? positiveAngle(target.getHeading() - follower.getPose().getHeading())
                    : -positiveAngle(follower.getPose().getHeading() - target.getHeading());
            follower.turn(rotation);
        } else if (step == RedAutoPlan.Step.FLOWER_DOWN) {
            // Preflight requires both independent presets. Never substitute a guessed endpoint.
            flower.lower();
            flower2.setPosition(settings.flower2Lowered);
            flowerStatus = "DOWN COMMANDED - elapsed settling only, no position feedback";
        }
    }

    private void updateMotion(long now) {
        double limit = RedAutoPlan.isTurn(step) ? settings.turnTimeout : settings.pathTimeout;
        require(elapsed(now, stepStarted) < limit, "Motion timeout at " + step + "; destination NOT accepted");
        if (!motionEnded && (distance(pose, progressPose) >= 0.10 || angleError(pose, progressPose) >= Math.toRadians(1.0))) {
            progressPose = pose;
            lastProgress = now;
        }
        require(motionEnded || elapsed(now, lastProgress) < settings.noProgressTimeout, "No localization progress at " + step);
        if (!motionEnded && !follower.isBusy()) {
            // Pedro can become idle on endpoint timeout or stuck detection. Never accept idle alone.
            stopMotion(); // Cancels the powered hold that Pedro retains after turns.
            require(poseAtDestination() && (activePath == null || activePath.getClosestPointTValue() >= 0.995),
                    "Pedro ended before verified destination/path end at " + step);
            motionEnded = true;
        }
        if (motionEnded) {
            require(poseAtDestination(), "Robot drifted outside destination tolerance while stopping");
            // Pedro's turn completion uses heading only. Wait for measured settling with output
            // cancelled, under the same finite deadline, without reissuing or restarting the turn.
            if (!atDestination()) return;
            intake.stop();
            advance(now);
        }
    }

    private void updateShooting(long now) {
        feedReady = false;
        require(atDestination(), "Robot drifted/moved during stationary shooting");
        FeedSettings feed = feedSettings();
        require(elapsed(now, stepStarted) < feed.timeoutSeconds, "Shooting timeout; ball count remains UNVERIFIED");
        if (feeding) {
            // Hold this attempt's previously confirmed preset while a ball traverses/leaves the sensor.
            // UNKNOWN is NOT a new identification. A different known ball aborts before servo switching.
            BallColorSensor.BallType detected = color.getBallType();
            require(detected == BallColorSensor.BallType.UNKNOWN || detected == attemptBall,
                    "Different ball detected during feed pulse; staging needs verification");
            require(atSpeed && shooter.isAdjustmentSettled(), "Readiness lost during feed attempt");
            if (elapsed(now, feedStarted) >= feed.onSeconds) {
                transfer.stop();
                feeding = false;
                attemptsCompleted++;
                gapStarted = now;
                shooter.setAutoMode(); // Next attempt needs a new current classification and readiness.
                feedStatus = "Attempt finished; fired-ball count UNVERIFIED";
            }
        } else {
            shooter.update(color.getBallType());
            feedReady = shooter.isReadyToFeed();
            if (elapsed(now, gapStarted) < feed.gapSeconds) return;
            if (attemptsCompleted == feed.attempts) {
                shooter.stop();
                advance(now);
            } else if (feedReady) {
                attemptBall = color.getBallType();
                require(attemptBall != BallColorSensor.BallType.UNKNOWN, "No current ball identification");
                shooter.setSelectionMode(Shooter.SelectionMode.MANUAL); // Holds the validated preset, no extra command.
                transfer.feed();
                feeding = true;
                feedStarted = now;
                feedStatus = "TIMED FEED ATTEMPT (not an exit/count measurement)";
            } else feedStatus = "WAITING FOR SPEED / BALL / SERVO";
        }
    }

    private void updateFlower(long now) {
        require(atDestination(), "Robot moved during flower action");
        require(elapsed(now, stepStarted) < settings.flowerTimeout, "Flower action timeout");
        if (elapsed(now, stepStarted) >= settings.flowerSettlingSeconds) advance(now);
    }

    private void advance(long now) {
        if (step == RedAutoPlan.Step.SHOOT_3) {
            running = false;
            status = "COMPLETE - all configured feed attempts ended; ball counts UNVERIFIED";
            shutdown();
            requestOpModeStop();
        } else enter(RedAutoPlan.Step.values()[step.ordinal() + 1], now);
    }

    private boolean atDestination() {
        return poseAtDestination() && Double.isFinite(speed) && speed <= settings.stoppedSpeed
                && Double.isFinite(angularSpeed) && Math.abs(angularSpeed) <= settings.stoppedAngularSpeed;
    }

    private boolean poseAtDestination() {
        return distance(pose, target) <= settings.positionTolerance && angleError(pose, target) <= settings.headingTolerance;
    }

    private FeedSettings feedSettings() {
        return step == RedAutoPlan.Step.SHOOT_1 ? settings.shoot1 : step == RedAutoPlan.Step.SHOOT_2 ? settings.shoot2 : settings.shoot3;
    }

    private void stopMotion() {
        // Mecanum.breakFollowing writes zero but can throw before all motors are reached.
        try { follower.breakFollowing(); }
        finally { driveTrain.stop(); }
    }

    private void fail(String context, RuntimeException failure) {
        running = prepared = false;
        status = "FAILED - restart OpMode only after resolving fault";
        error = context + ": " + failure.getMessage();
        RobotLog.ee("RedAuto", failure, error);
        shutdown();
    }

    @Override public void stop() { running = prepared = false; shutdown(); }

    private void shutdown() {
        if (shutdownDone) return;
        shutdownDone = true;
        // No follower updates after this latch, even if cancellation itself fails. No servo movement.
        safeStop("Pedro", () -> { if (follower != null) follower.breakFollowing(); });
        safeStop("drivetrain", () -> { if (driveTrain != null) driveTrain.stop(); });
        safeStop("intake", () -> { if (intake != null) intake.stop(); });
        safeStop("transfer", () -> { if (transfer != null) transfer.stop(); });
        safeStop("flywheels", () -> { if (shooter != null) shooter.stop(); });
        // Also covers a constructor/factory throwing before its owner reference was assigned.
        for (int i = 0; i < motors.length; i++) {
            DcMotorEx motor = motors[i];
            if (motor != null) safeStop(MOTOR_NAMES[i], () -> motor.setPower(0.0));
        }
        atSpeed = feeding = feedReady = false;
        feedStatus = "STOPPED; ball count UNVERIFIED";
    }

    private void safeStop(String name, Runnable action) {
        try { action.run(); }
        catch (RuntimeException e) {
            error += "; stop failed: " + name;
            RobotLog.ee("RedAuto", e, "Stop failed: " + name);
        }
    }

    private void validateWiring() {
        // Relative hub identity is checked; physical Control/Expansion labels require RC setup review.
        require(motors[4].getController() == motors[1].getController() && motors[4].getPortNumber() == 2,
                "intake must be right/Control Hub motor 2");
        require(motors[7].getController() == motors[1].getController() && motors[7].getPortNumber() == 3,
                "flywheelB must be right/Control Hub motor+encoder 3");
        require(motors[6].getController() == motors[0].getController() && motors[6].getPortNumber() == 2,
                "flywheelA must be left/Expansion Hub motor+encoder 2");
        for (int i = 0; i < motors.length; i++) for (int j = i + 1; j < motors.length; j++) {
            require(motors[i] != motors[j] && !(motors[i].getController() == motors[j].getController()
                    && motors[i].getPortNumber() == motors[j].getPortNumber()), "Duplicate motor channel ownership");
        }
        Servo first = hardwareMap.get(Servo.class, "flowerServo");
        Servo second = hardwareMap.get(Servo.class, "flowerServo2");
        Servo adjust = hardwareMap.get(Servo.class, "shooterAdjust");
        require(first.getPortNumber() == 0 && second.getPortNumber() == 0 && adjust.getPortNumber() == 1
                && first.getController() != second.getController() && second.getController() == adjust.getController(),
                "Expected flowerServo on Control servo 0; flowerServo2/shooterAdjust on Expansion servo 0/1");
    }

    private List<String> configurationIssues() {
        List<String> issues = new ArrayList<>();
        if (!settings.routeAndMechanismsVerified) issues.add("Route, obstacle clearance, dual flower and feed timing not physically verified");
        if (!Double.isFinite(settings.startX) || !Double.isFinite(settings.startY)) issues.add("Measured starting X/Y missing");
        if (!positive(settings.fieldWidth) || !positive(settings.fieldHeight)) issues.add("Verified field width/height missing");
        if (!positive(settings.sweptRobotRadius)) issues.add("Swept robot/flower clearance radius missing");
        if (!Double.isFinite(settings.boundaryMargin) || settings.boundaryMargin < settings.positionTolerance)
            issues.add("Boundary margin missing/insufficient (must cover tracking tolerance plus stopping clearance)");
        if (!positive(settings.intake2Distance)) issues.add("INTAKE_2 measured distance missing");
        for (double value : new double[]{settings.pathPower, settings.positionTolerance, settings.headingTolerance,
                settings.stoppedSpeed, settings.stoppedAngularSpeed, settings.pathTimeout, settings.turnTimeout,
                settings.noProgressTimeout, settings.endpointTimeoutMs, settings.maxLoopGap, settings.autonomousDeadline,
                settings.intakePower, settings.transferPower, settings.flywheelTolerance, settings.flywheelStableSeconds,
                settings.flowerTimeout}) {
            if (!positive(value)) { issues.add("Software motion/mechanism limits must be finite and positive"); break; }
        }
        if (settings.pathPower > 1 || settings.intakePower > 1 || settings.transferPower > 1
                || settings.headingTolerance >= Math.PI / 2 || settings.flywheelTolerance >= SHOOTING_TICKS_PER_SECOND
                || settings.autonomousDeadline > 30.0) issues.add("Power, readiness, angle or autonomous deadline limit out of range");
        if (!Double.isFinite(settings.pathTimeout * 1000.0) || settings.endpointTimeoutMs <= settings.pathTimeout * 1000.0)
            issues.add("Pedro endpoint timeout must exceed the whole-path watchdog");
        if (!position(settings.pollenPosition) || !position(settings.nectarPosition)
                || !positive(settings.adjustmentSettlingSeconds)) issues.add("Shooter adjustment calibration invalid");
        if (!position(settings.flower1Raised) || !position(settings.flower1Lowered)) issues.add("flowerServo endpoints missing/invalid");
        if (!position(settings.flower2Lowered) || settings.flower2Direction == null) issues.add("flowerServo2 lowered endpoint/direction missing");
        if (!positive(settings.flowerSettlingSeconds) || settings.flowerSettlingSeconds >= settings.flowerTimeout)
            issues.add("Dual flower settling time missing/invalid (must be less than timeout)");
        FeedSettings[] feeds = {settings.shoot1, settings.shoot2, settings.shoot3};
        for (int i = 0; i < feeds.length; i++) {
            FeedSettings f = feeds[i];
            if (f.attempts <= 0 || !positive(f.onSeconds) || !positive(f.gapSeconds) || !positive(f.timeoutSeconds)
                    || f.attempts * (f.onSeconds + f.gapSeconds) + f.gapSeconds >= f.timeoutSeconds) {
                issues.add("SHOOT_" + (i + 1) + " requires valid attempt count, pulse/gap duration and a sufficient finite timeout");
            }
        }
        if (settings.shoot1.attempts != 4 || settings.shoot2.attempts != 4) issues.add("SHOOT_1 and SHOOT_2 require four feed attempts each");
        return issues;
    }

    private void showTelemetry() {
        telemetry.addData("Auto", status);
        telemetry.addData("State 1-16", step == null ? "NOT STARTED" : (step.ordinal() + 1) + "/16 " + step);
        telemetry.addData("Pedro X/Y/heading", poseText(pose));
        telemetry.addData("Target X/Y/heading", poseText(target));
        telemetry.addData("Path progress t", activePath == null ? "N/A (turn/action)" : activePath.getClosestPointTValue());
        telemetry.addData("Follower", follower == null ? "NOT INITIALIZED" : running && follower.isBusy() ? "BUSY" : "IDLE/STOPPED");
        telemetry.addData("Localization", localizer == null ? "UNCONFIGURED" : localizer.getStatus());
        telemetry.addData("Flywheel A/B ticks/s", "%.1f / %.1f", velocityA, velocityB);
        telemetry.addData("Flywheel target / at speed", "%.0f / %s", shooter == null ? 0.0 : shooter.getTargetVelocity(), atSpeed);
        telemetry.addData("Intake / transfer", (intake == null ? "UNINITIALIZED" : intake.getState()) + " / "
                + (transfer == null ? "UNINITIALIZED" : transfer.getState()));
        telemetry.addData("Feed", feedStatus);
        telemetry.addData("Completed feed attempts (NOT ball count)", attemptsCompleted);
        telemetry.addData("Ready to begin feed", feedReady && !feeding);
        telemetry.addData("Flower", flowerStatus);
        telemetry.addData("Color", color == null ? "UNAVAILABLE" : color.getBallType() + " / " + color.getStatus());
        telemetry.addData("Color calibration", color == null ? "UNAVAILABLE" : color.getCalibrationStatus());
        telemetry.addData("Errors / missing configuration", error.isEmpty() ? "None" : error);
        telemetry.addLine("B on either gamepad aborts. Feed attempts and servo settling are not physical confirmation.");
        telemetry.update();
    }

    private static double elapsed(long now, long start) { return (now - start) / 1_000_000_000.0; }
    private static double positiveAngle(double angle) { return (angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI); }
    private static double distance(Pose a, Pose b) { return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY()); }
    private static double angleError(Pose a, Pose b) { return Math.abs(Math.atan2(Math.sin(a.getHeading() - b.getHeading()), Math.cos(a.getHeading() - b.getHeading()))); }
    private static boolean positive(double value) { return Double.isFinite(value) && value > 0.0; }
    private static boolean position(double value) { return Double.isFinite(value) && value >= 0.0 && value <= 1.0; }
    private static String poseText(Pose p) { return p == null ? "UNKNOWN" : String.format("%.2f / %.2f / %.1f deg", p.getX(), p.getY(), Math.toDegrees(p.getHeading())); }
    private static void require(boolean condition, String message) { if (!condition) throw new IllegalStateException(message); }
}
