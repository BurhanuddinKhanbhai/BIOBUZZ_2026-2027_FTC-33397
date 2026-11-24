
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
 * BIOBUZZ Red Autonomous.
 *
 * Starting pose: (0, 0, 0).
 * All movements are relative to the starting position.
 *
 * Shooting:
 * - Pollen: 1100 ticks/s
 * - Nectar: 1200 ticks/s
 * - Intake: 100%
 * - Transfer: 90%
 * - Both feed motors run together for 4 seconds
 *
 * Each SHOOT step executes one continuous feeding burst.
 * The burst does not count individual balls.
 *
 * Field boundaries are not checked by this program.
 */
@Autonomous(
        name = "BIOBUZZ - Red Auto",
        group = "Competition"
)
public class RedAuto extends OpMode {

    public static final class Settings {

        // Robot-relative movement
        public double intake2Distance = 48.0;

        // Pedro movement settings
        public double pathPower = 0.25;
        public double positionTolerance = 1.0;
        public double headingTolerance = Math.toRadians(3.0);

        public double stoppedSpeed = 1.0;
        public double stoppedAngularSpeed = 0.10;

        public double pathTimeout = 10.0;
        public double turnTimeout = 6.0;
        public double noProgressTimeout = 2.0;

        public double endpointTimeoutMs = 11000.0;
        public double maxLoopGap = 0.25;

        // Total autonomous time limit
        public double autonomousDeadline = 29.0;

        // Intake and transfer
        public double intakePower = 1.0;
        public double transferPower = 0.90;

        // Continuous feeding burst
        public double feedDurationSeconds = 4.0;

        // Includes waiting for flywheel speed and feeding
        public double shootTimeoutSeconds = 12.0;

        // Shooter readiness
        public double flywheelTolerance = 75.0;
        public double flywheelStableSeconds = 0.25;

        // Shooter adjustment servo
        public double pollenPosition = 0.10;
        public double nectarPosition = 0.00;
        public double adjustmentSettlingSeconds = 0.05;

        // Flower arms
        public double flower1Raised = 0.225;
        public double flower1Lowered = 0.48;

        public double flower2Lowered = 0.48;
        public Servo.Direction flower2Direction =
                Servo.Direction.REVERSE;

        public double flowerSettlingSeconds = 1.0;
        public double flowerTimeout = 5.0;
    }

    private static final double POLLEN_TICKS_PER_SECOND = 1100.0;
    private static final double NECTAR_TICKS_PER_SECOND = 1200.0;

    private static final String[] MOTOR_NAMES = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight",
            "intake",
            "transfer",
            "flywheelA",
            "flywheelB"
    };

    private final Settings settings;
    private final LongSupplier clock;

    private final DcMotorEx[] motors =
            new DcMotorEx[MOTOR_NAMES.length];

    private RedAutoPlan plan;
    private RedAutoPlan.Step step;

    private Follower follower;
    private SafeThreeWheelLocalizer localizer;

    private DriveTrain driveTrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private FlowerMechanism flower;
    private Servo flower2;
    private BallColorSensor color;

    private Path activePath;

    private Pose target;
    private Pose pose;
    private Pose progressPose;

    private boolean prepared;
    private boolean running;
    private boolean started;
    private boolean poseGuardArmed;
    private boolean feeding;
    private boolean shutdownDone;
    private boolean motionEnded;

    private boolean atSpeed;
    private boolean feedReady;

    private long autoStarted;
    private long stepStarted;
    private long lastLoop;
    private long lastProgress;
    private long feedStarted;

    private double velocityA = Double.NaN;
    private double velocityB = Double.NaN;
    private double speed = Double.NaN;
    private double angularSpeed = Double.NaN;

    private String status = "NOT INITIALIZED";
    private String error = "";
    private String feedStatus = "STOPPED";
    private String flowerStatus = "UNCOMMANDED";

    private BallColorSensor.BallType feedingBall =
            BallColorSensor.BallType.UNKNOWN;

    public RedAuto() {
        this(new Settings(), System::nanoTime);
    }

    RedAuto(Settings settings, LongSupplier clock) {
        this.settings = settings;
        this.clock = clock;
    }

    @Override
    public void init() {
        List<String> issues = configurationIssues();

        String pedroIssue = Constants.getConfigurationIssue();
        if (pedroIssue != null) {
            issues.add(pedroIssue);
        }

        try {
            plan = new RedAutoPlan(settings.intake2Distance);
        } catch (IllegalArgumentException e) {
            issues.add(e.getMessage());
        }

        try {
            for (int i = 0; i < motors.length; i++) {
                motors[i] = hardwareMap.tryGet(
                        DcMotorEx.class,
                        MOTOR_NAMES[i]
                );

                if (motors[i] == null) {
                    issues.add("Missing motor: " + MOTOR_NAMES[i]);
                }
            }

            for (String name : new String[]{
                    "shooterAdjust",
                    "flowerServo",
                    "flowerServo2"
            }) {
                if (hardwareMap.tryGet(Servo.class, name) == null) {
                    issues.add("Missing servo: " + name);
                }
            }

            color = new BallColorSensor(hardwareMap);

            if (!color.isAvailable()) {
                issues.add("Missing ballColor sensor");
            }

            if (!color.isCalibrated()) {
                issues.add(
                        "Color calibration: "
                                + color.getCalibrationStatus()
                );
            }

            if (!color.isRedCalibrated()) {
                issues.add("Red nectar calibration missing");
            }

        } catch (RuntimeException e) {
            issues.add(
                    "Hardware lookup/calibration: "
                            + e.getMessage()
            );
        }

        // Stop existing motor output without resetting encoders.
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] == null) {
                continue;
            }

            try {
                motors[i].setPower(0.0);
            } catch (RuntimeException e) {
                issues.add(
                        "Initial stop failed: " + MOTOR_NAMES[i]
                );

                RobotLog.ee(
                        "RedAuto",
                        e,
                        "Initial stop failed: " + MOTOR_NAMES[i]
                );
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

            driveTrain = new DriveTrain(hardwareMap);

            intake = new Intake(
                    hardwareMap,
                    DcMotorSimple.Direction.REVERSE,
                    settings.intakePower
            );

            transfer = new Transfer(
                    hardwareMap,
                    DcMotorSimple.Direction.REVERSE,
                    settings.transferPower
            );

            shooter = new Shooter(
                    hardwareMap,
                    DcMotorSimple.Direction.FORWARD,
                    DcMotorSimple.Direction.REVERSE,
                    settings.flywheelTolerance,
                    settings.flywheelStableSeconds,
                    Servo.Direction.FORWARD,
                    settings.adjustmentSettlingSeconds
            );

            shooter.configureAdjustmentPositions(
                    settings.pollenPosition,
                    settings.nectarPosition
            );

            shooter.setAutoMode();

            flower = new FlowerMechanism(hardwareMap);
            flower.configurePositions(
                    settings.flower1Raised,
                    settings.flower1Lowered
            );

            flower2 = hardwareMap.get(
                    Servo.class,
                    "flowerServo2"
            );

            flower2.setDirection(settings.flower2Direction);

            flowerStatus = "Presets configured; UNCOMMANDED";

            follower = Constants.createFollower(
                    hardwareMap,
                    next -> {
                        if (poseGuardArmed && !plan.contains(next)) {
                            throw new IllegalStateException(
                                    "Invalid/nonfinite relative localization pose"
                            );
                        }
                    }
            );

            localizer = (SafeThreeWheelLocalizer)
                    follower.getPoseTracker().getLocalizer();

            follower.setStartingPose(plan.getStart());
            pose = plan.getStart();

            prepared = true;
            status = "READY - configuration passed; field clearance unchecked";

        } catch (RuntimeException e) {
            fail("Initialization failed", e);
        }

        showTelemetry();
    }

    @Override
    public void init_loop() {
        showTelemetry();
    }

    @Override
    public void start() {
        if (started || !prepared || shutdownDone) {
            return;
        }

        started = true;

        try {
            require(
                    !gamepad1.b && !gamepad2.b,
                    "Operator emergency stop held at START (B)"
            );

            // Robot's current position becomes (0, 0, 0).
            follower.setStartingPose(plan.getStart());

            poseGuardArmed = true;

            autoStarted = clock.getAsLong();
            lastLoop = autoStarted;
            running = true;

            enter(RedAutoPlan.Step.SHOOT_1, autoStarted);

        } catch (RuntimeException e) {
            fail("Start failed", e);
        }
    }

    @Override
    public void loop() {
        if (running) {
            try {
                long now = clock.getAsLong();

                require(
                        !gamepad1.b && !gamepad2.b,
                        "Operator emergency stop (B)"
                );

                require(
                        elapsed(now, autoStarted)
                                < settings.autonomousDeadline,
                        "Autonomous deadline expired"
                );

                require(
                        now >= lastLoop
                                && elapsed(now, lastLoop)
                                <= settings.maxLoopGap,
                        "Control-loop gap exceeded safety limit"
                );

                double actionLimit =
                        RedAutoPlan.isDrive(step)
                                ? settings.pathTimeout
                                : RedAutoPlan.isTurn(step)
                                  ? settings.turnTimeout
                                  : RedAutoPlan.isShoot(step)
                                    ? settings.shootTimeoutSeconds
                                    : settings.flowerTimeout;

                require(
                        elapsed(now, stepStarted) < actionLimit,
                        "Action timeout at " + step
                                + "; NOT completed"
                );

                lastLoop = now;

                follower.update();

                pose = follower.getPose();
                speed = follower.getVelocity().getMagnitude();
                angularSpeed = follower.getAngularVelocity();

                require(
                        localizer.isSampleValid()
                                && !follower.isLocalizationNAN()
                                && Double.isFinite(speed)
                                && Double.isFinite(angularSpeed),
                        "Invalid localization/velocity: "
                                + localizer.getStatus()
                );

                color.update();

                require(
                        color.hasColorSample(),
                        "Color sensor unavailable/invalid: "
                                + color.getStatus()
                );

                // Select speed before feeding begins.
                // Keep that speed during the four-second burst.
                if (RedAutoPlan.isShoot(step) && !feeding) {
                    updateFlywheelTargetForBall();
                }

                velocityA = shooter.getVelocityA();
                velocityB = shooter.getVelocityB();

                require(
                        Double.isFinite(velocityA)
                                && Double.isFinite(velocityB),
                        "Invalid flywheel velocity feedback"
                );

                atSpeed = shooter.isAtSpeed();

                if (RedAutoPlan.isDrive(step)
                        || RedAutoPlan.isTurn(step)) {
                    updateMotion(now);

                } else if (RedAutoPlan.isShoot(step)) {
                    updateShooting(now);

                } else {
                    updateFlower(now);
                }

            } catch (RuntimeException e) {
                fail("Autonomous stopped", e);
            }
        }

        showTelemetry();
    }

    /**
     * Select flywheel target based on the detected ball.
     * Unknown colour does not start feeding.
     */
    private void updateFlywheelTargetForBall() {
        BallColorSensor.BallType detected = color.getBallType();

        if (detected == BallColorSensor.BallType.POLLEN) {
            shooter.setTargetVelocity(POLLEN_TICKS_PER_SECOND);

        } else if (detected == BallColorSensor.BallType.NECTAR) {
            shooter.setTargetVelocity(NECTAR_TICKS_PER_SECOND);
        }
    }

    private void enter(RedAutoPlan.Step next, long now) {
        step = next;
        target = plan.getTarget(step);

        stepStarted = now;
        lastProgress = now;
        progressPose = follower.getPose();

        activePath = null;
        motionEnded = false;

        status = "RUNNING";

        // Intake only while collecting, until a shooting burst starts.
        if (step == RedAutoPlan.Step.INTAKE_1
                || step == RedAutoPlan.Step.INTAKE_2) {
            intake.intake();
        } else {
            intake.stop();
        }

        transfer.stop();

        feeding = false;
        feedReady = false;
        atSpeed = false;

        if (RedAutoPlan.isShoot(step)) {
            shooter.setAutoMode();
            shooter.stop();

            feedingBall = BallColorSensor.BallType.UNKNOWN;
            feedStatus = "WAITING FOR SPEED / BALL / SERVO";

        } else {
            shooter.stop();
            feedStatus = "STOPPED";
        }

        if (RedAutoPlan.isDrive(step)) {
            Pose pathStart = step.ordinal() == 0
                    ? plan.getStart()
                    : plan.getTarget(
                    RedAutoPlan.Step.values()[step.ordinal() - 1]
            );

            activePath = new Path(
                    new BezierLine(pathStart, target),
                    new PathConstraints(
                            0.995,
                            settings.stoppedSpeed,
                            settings.positionTolerance,
                            settings.headingTolerance,
                            settings.endpointTimeoutMs,
                            Constants.pathConstraints.getBrakingStrength(),
                            Constants.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(),
                            Constants.pathConstraints.getBrakingStart()
                    )
            );

            activePath.setConstantHeadingInterpolation(
                    target.getHeading()
            );

            follower.setMaxPower(
                    Math.min(
                            settings.pathPower,
                            Constants.mecanumMaxPower
                    )
            );

            follower.followPath(activePath, false);

        } else if (RedAutoPlan.isTurn(step)) {
            follower.setMaxPower(
                    Math.min(
                            settings.pathPower,
                            Constants.mecanumMaxPower
                    )
            );

            double rotation =
                    RedAutoPlan.turnRadians(step) > 0.0
                            ? positiveAngle(
                            target.getHeading()
                            - follower.getPose().getHeading()
                    )
                            : -positiveAngle(
                            follower.getPose().getHeading()
                            - target.getHeading()
                    );

            follower.turn(rotation);

        } else if (step == RedAutoPlan.Step.FLOWER_DOWN) {
            flower.lower();
            flower2.setPosition(settings.flower2Lowered);

            flowerStatus = "DOWN COMMANDED; awaiting settling";
        }
    }

    private void updateMotion(long now) {
        double limit = RedAutoPlan.isTurn(step)
                ? settings.turnTimeout
                : settings.pathTimeout;

        require(
                elapsed(now, stepStarted) < limit,
                "Motion timeout at " + step
                        + "; destination NOT accepted"
        );

        if (!motionEnded
                && (distance(pose, progressPose) >= 0.10
                || angleError(pose, progressPose)
                >= Math.toRadians(1.0))) {
            progressPose = pose;
            lastProgress = now;
        }

        require(
                motionEnded
                        || elapsed(now, lastProgress)
                        < settings.noProgressTimeout,
                "No localization progress at " + step
        );

        if (!motionEnded && !follower.isBusy()) {
            stopMotion();

            require(
                    poseAtDestination()
                            && (activePath == null
                            || activePath.getClosestPointTValue() >= 0.995),
                    "Pedro ended before verified destination/path end at "
                            + step
            );

            motionEnded = true;
        }

        if (motionEnded) {
            require(
                    poseAtDestination(),
                    "Robot drifted outside destination tolerance"
            );

            if (!atDestination()) {
                return;
            }

            intake.stop();
            advance(now);
        }
    }

    /**
     * One continuous four-second feeding burst.
     *
     * Wait for the first detected ball's proper speed and
     * servo position, then run intake and transfer together.
     *
     * This does not measure how many balls are actually shot.
     */
    private void updateShooting(long now) {
        feedReady = false;

        require(
                atDestination(),
                "Robot drifted/moved during stationary shooting"
        );

        if (feeding) {
            double feedElapsed = elapsed(now, feedStarted);

            if (feedElapsed >= settings.feedDurationSeconds) {
                // End the continuous four-second burst.
                intake.stop();
                transfer.stop();

                feeding = false;
                shooter.stop();

                feedStatus =
                        "4-second feeding complete; ball count UNVERIFIED";

                // Move to the next autonomous step.
                advance(now);
            } else {
                feedStatus = String.format(
                        "FEEDING %.2f / %.2f seconds (%s)",
                        feedElapsed,
                        settings.feedDurationSeconds,
                        feedingBall
                );
            }

            return;
        }

        // Before starting the burst, wait for the
        // selected ball, flywheel speed, and servo position.
        shooter.update(color.getBallType());

        feedReady = shooter.isReadyToFeed();

        if (!feedReady) {
            feedStatus = "WAITING FOR SPEED / BALL / SERVO";
            return;
        }

        feedingBall = color.getBallType();

        require(
                feedingBall != BallColorSensor.BallType.UNKNOWN,
                "No current ball identification"
        );

        // Hold the detected ball's adjustment position
        // throughout this feeding burst.
        shooter.setSelectionMode(
                Shooter.SelectionMode.MANUAL
        );

        // Start both motors at the same time.
        // Intake = 100%; transfer = 90%.
        intake.intake();
        transfer.feed();

        feeding = true;
        feedStarted = now;

        feedStatus = "FEEDING - both motors ON for 4 seconds";
    }

    private void updateFlower(long now) {
        require(
                atDestination(),
                "Robot moved during flower action"
        );

        require(
                elapsed(now, stepStarted)
                        < settings.flowerTimeout,
                "Flower action timeout"
        );

        if (elapsed(now, stepStarted)
                >= settings.flowerSettlingSeconds) {
            advance(now);
        }
    }

    private void advance(long now) {
        if (step == RedAutoPlan.Step.SHOOT_3) {
            running = false;

            status =
                    "COMPLETE - feeding ended; ball counts UNVERIFIED";

            shutdown();
            requestOpModeStop();

        } else {
            enter(
                    RedAutoPlan.Step.values()[step.ordinal() + 1],
                    now
            );
        }
    }

    private boolean atDestination() {
        return poseAtDestination()
                && Double.isFinite(speed)
                && speed <= settings.stoppedSpeed
                && Double.isFinite(angularSpeed)
                && Math.abs(angularSpeed)
                <= settings.stoppedAngularSpeed;
    }

    private boolean poseAtDestination() {
        return distance(pose, target)
                <= settings.positionTolerance
                && angleError(pose, target)
                <= settings.headingTolerance;
    }

    private void stopMotion() {
        try {
            follower.breakFollowing();
        } finally {
            driveTrain.stop();
        }
    }

    private void fail(String context, RuntimeException failure) {
        running = false;
        prepared = false;

        status = "FAILED - restart OpMode after resolving fault";
        error = context + ": " + failure.getMessage();

        RobotLog.ee("RedAuto", failure, error);

        shutdown();
    }

    @Override
    public void stop() {
        running = false;
        prepared = false;
        shutdown();
    }

    private void shutdown() {
        if (shutdownDone) {
            return;
        }

        shutdownDone = true;

        safeStop("Pedro", () -> {
            if (follower != null) {
                follower.breakFollowing();
            }
        });

        safeStop("drivetrain", () -> {
            if (driveTrain != null) {
                driveTrain.stop();
            }
        });

        safeStop("intake", () -> {
            if (intake != null) {
                intake.stop();
            }
        });

        safeStop("transfer", () -> {
            if (transfer != null) {
                transfer.stop();
            }
        });

        safeStop("flywheels", () -> {
            if (shooter != null) {
                shooter.stop();
            }
        });

        // Independently stop all mapped motors.
        for (int i = 0; i < motors.length; i++) {
            DcMotorEx motor = motors[i];

            if (motor != null) {
                safeStop(
                        MOTOR_NAMES[i],
                        () -> motor.setPower(0.0)
                );
            }
        }

        atSpeed = false;
        feeding = false;
        feedReady = false;

        feedStatus = "STOPPED; ball count UNVERIFIED";
    }

    private void safeStop(String name, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException e) {
            error += "; stop failed: " + name;

            RobotLog.ee(
                    "RedAuto",
                    e,
                    "Stop failed: " + name
            );
        }
    }

    private void validateWiring() {
        require(
                motors[4].getController()
                        == motors[1].getController()
                        && motors[4].getPortNumber() == 2,
                "intake must be Control Hub motor 2"
        );

        require(
                motors[7].getController()
                        == motors[1].getController()
                        && motors[7].getPortNumber() == 3,
                "flywheelB must be Control Hub motor 3"
        );

        require(
                motors[6].getController()
                        == motors[0].getController()
                        && motors[6].getPortNumber() == 2,
                "flywheelA must be Expansion Hub motor 2"
        );

        for (int i = 0; i < motors.length; i++) {
            for (int j = i + 1; j < motors.length; j++) {
                require(
                        motors[i] != motors[j]
                                && !(motors[i].getController()
                                == motors[j].getController()
                                && motors[i].getPortNumber()
                                == motors[j].getPortNumber()),
                        "Duplicate motor channel ownership"
                );
            }
        }

        Servo first = hardwareMap.get(
                Servo.class, "flowerServo"
        );

        Servo second = hardwareMap.get(
                Servo.class, "flowerServo2"
        );

        Servo adjust = hardwareMap.get(
                Servo.class, "shooterAdjust"
        );

        require(
                first.getPortNumber() == 0
                        && second.getPortNumber() == 0
                        && adjust.getPortNumber() == 1
                        && first.getController()
                        != second.getController()
                        && second.getController()
                        == adjust.getController(),
                "Flower servo port mapping does not match hardware"
        );
    }

    private List<String> configurationIssues() {
        List<String> issues = new ArrayList<>();

        if (!positive(settings.intake2Distance)) {
            issues.add("INTAKE_2 distance invalid");
        }

        for (double value : new double[]{
                settings.pathPower,
                settings.positionTolerance,
                settings.headingTolerance,
                settings.stoppedSpeed,
                settings.stoppedAngularSpeed,
                settings.pathTimeout,
                settings.turnTimeout,
                settings.noProgressTimeout,
                settings.endpointTimeoutMs,
                settings.maxLoopGap,
                settings.autonomousDeadline,
                settings.intakePower,
                settings.transferPower,
                settings.flywheelTolerance,
                settings.flywheelStableSeconds,
                settings.feedDurationSeconds,
                settings.shootTimeoutSeconds,
                settings.flowerTimeout
        }) {
            if (!positive(value)) {
                issues.add(
                        "Motion/mechanism limits must be finite and positive"
                );
                break;
            }
        }

        if (settings.pathPower > 1.0
                || settings.intakePower > 1.0
                || settings.transferPower > 1.0
                || settings.headingTolerance >= Math.PI / 2
                || settings.flywheelTolerance
                >= POLLEN_TICKS_PER_SECOND
                || settings.autonomousDeadline > 30.0) {
            issues.add(
                    "Power, readiness, angle or deadline out of range"
            );
        }

        if (!Double.isFinite(settings.pathTimeout * 1000.0)
                || settings.endpointTimeoutMs
                <= settings.pathTimeout * 1000.0) {
            issues.add(
                    "Pedro endpoint timeout must exceed path watchdog"
            );
        }

        if (!position(settings.pollenPosition)
                || !position(settings.nectarPosition)
                || !positive(settings.adjustmentSettlingSeconds)) {
            issues.add("Shooter servo calibration invalid");
        }

        if (!position(settings.flower1Raised)
                || !position(settings.flower1Lowered)) {
            issues.add("Flower servo positions invalid");
        }

        if (!position(settings.flower2Lowered)
                || settings.flower2Direction == null) {
            issues.add("Second flower servo calibration invalid");
        }

        if (!positive(settings.flowerSettlingSeconds)
                || settings.flowerSettlingSeconds
                >= settings.flowerTimeout) {
            issues.add("Flower settling time invalid");
        }

        if (settings.feedDurationSeconds
                >= settings.shootTimeoutSeconds) {
            issues.add(
                    "Feed duration must be below shooting timeout"
            );
        }

        // Lower-bound timing check.
        // Real shooting and movement will take longer.
        double minimumScheduledSeconds =
                3.0 * settings.feedDurationSeconds
                        + settings.flowerSettlingSeconds;

        if (minimumScheduledSeconds
                >= settings.autonomousDeadline) {
            issues.add(
                    "Minimum feeding and flower duration exceeds deadline"
            );
        }

        return issues;
    }

    private void showTelemetry() {
        telemetry.addData("Auto", status);

        telemetry.addData(
                "State 1-16",
                step == null
                        ? "NOT STARTED"
                        : (step.ordinal() + 1) + "/16 " + step
        );

        telemetry.addData(
                "Pedro X/Y/heading",
                poseText(pose)
        );

        telemetry.addData(
                "Target X/Y/heading",
                poseText(target)
        );

        telemetry.addData(
                "Path progress t",
                activePath == null
                        ? "N/A (turn/action)"
                        : activePath.getClosestPointTValue()
        );

        telemetry.addData(
                "Follower",
                follower == null
                        ? "NOT INITIALIZED"
                        : running && follower.isBusy()
                          ? "BUSY"
                          : "IDLE/STOPPED"
        );

        telemetry.addData(
                "Localization",
                localizer == null
                        ? "UNCONFIGURED"
                        : localizer.getStatus()
        );

        telemetry.addData(
                "Flywheel A/B ticks/s",
                "%.1f / %.1f",
                velocityA,
                velocityB
        );

        telemetry.addData(
                "Flywheel target / at speed",
                "%.0f / %s",
                shooter == null
                        ? 0.0
                        : shooter.getTargetVelocity(),
                atSpeed
        );

        telemetry.addData(
                "Intake / transfer",
                (intake == null
                        ? "UNINITIALIZED"
                        : intake.getState())
                        + " / "
                        + (transfer == null
                        ? "UNINITIALIZED"
                        : transfer.getState())
        );

        telemetry.addData("Feed", feedStatus);

        telemetry.addData(
                "Ready to begin feed",
                feedReady && !feeding
        );

        telemetry.addData("Flower", flowerStatus);

        telemetry.addData(
                "Color",
                color == null
                        ? "UNAVAILABLE"
                        : color.getBallType()
                          + " / " + color.getStatus()
        );

        telemetry.addData(
                "Color calibration",
                color == null
                        ? "UNAVAILABLE"
                        : color.getCalibrationStatus()
        );

        telemetry.addData(
                "Errors / missing configuration",
                error.isEmpty() ? "None" : error
        );

        telemetry.addLine(
                "B on either gamepad aborts. Field clearance unchecked."
        );

        telemetry.update();
    }

    private static double elapsed(long now, long start) {
        return (now - start) / 1_000_000_000.0;
    }

    private static double positiveAngle(double angle) {
        return (angle % (2 * Math.PI) + 2 * Math.PI)
                % (2 * Math.PI);
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(
                a.getX() - b.getX(),
                a.getY() - b.getY()
        );
    }

    private static double angleError(Pose a, Pose b) {
        return Math.abs(
                Math.atan2(
                        Math.sin(a.getHeading() - b.getHeading()),
                        Math.cos(a.getHeading() - b.getHeading())
                )
        );
    }

    private static boolean positive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    private static boolean position(double value) {
        return Double.isFinite(value)
                && value >= 0.0
                && value <= 1.0;
    }

    private static String poseText(Pose p) {
        return p == null
                ? "UNKNOWN"
                : String.format(
                "%.2f / %.2f / %.1f deg",
                p.getX(),
                p.getY(),
                Math.toDegrees(p.getHeading())
        );
    }

    private static void require(
            boolean condition,
            String message
    ) {
        if (!condition) {
            throw new IllegalStateException(message);
        }
    }
}
