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
import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.function.LongSupplier;

/** Sixteen robot-relative steps. Pedro controls movement; feed bursts do not count ball exits. */
@Autonomous(name = "BIOBUZZ - Red Auto", group = "Competition")
public class RedAuto extends OpMode {
    public static final class Settings {
        public double intake2Distance = 48.0;
        public double pathPower = 0.25;
        // Existing Pedro path-completion constraints, not additional OpMode watchdogs.
        public double positionTolerance = 1.0;
        public double headingTolerance = Math.toRadians(3.0);
        public double stoppedSpeed = 1.0;
        public double endpointTimeoutMs = 11000.0;
        public double intakePower = 1.0;
        public double transferPower = 0.90;
        public double feedDurationSeconds = 4.0;
        public double flywheelTolerance = 75.0;
        public double flywheelStableSeconds = 0.25;
        public double pollenPosition = 0.10;
        public double nectarPosition = 0.00;
        public double adjustmentSettlingSeconds = 0.05;
        public double flower1Raised = 0.225;
        public double flower1Lowered = 0.48;
        public double flower2Lowered = 0.48;
        public Servo.Direction flower2Direction = Servo.Direction.REVERSE;
        public double flowerSettlingSeconds = 1.0;
    }

    private static final double POLLEN_TICKS_PER_SECOND = 1100.0;
    private static final double NECTAR_TICKS_PER_SECOND = 1200.0;
    private final Settings settings;
    private final LongSupplier clock;
    private RedAutoPlan plan;
    private RedAutoPlan.Step step;
    private Follower follower;
    private DriveTrain driveTrain; // Initializes motor modes/directions before Pedro takes over.
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private FlowerMechanism flower;
    private Servo flower2;
    private BallColorSensor color;
    private Path activePath;
    private Pose target;
    private boolean running, feeding;
    private long stepStarted, feedStarted;
    private BallColorSensor.BallType feedingBall = BallColorSensor.BallType.UNKNOWN;
    private String status = "INITIALIZING";
    private String feedStatus = "STOPPED";

    public RedAuto() { this(new Settings(), System::nanoTime); }
    RedAuto(Settings settings, LongSupplier clock) { this.settings = settings; this.clock = clock; }

    @Override public void init() {
        plan = new RedAutoPlan(settings.intake2Distance);
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
        color = new BallColorSensor(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(plan.getStart());
        status = "READY";
        showTelemetry();
    }

    @Override public void init_loop() { showTelemetry(); }

    @Override public void start() {
        if (gamepad1.b || gamepad2.b) { stop(); requestOpModeStop(); return; }
        follower.setStartingPose(plan.getStart()); // Software count baseline, no encoder reset.
        running = true;
        enter(RedAutoPlan.Step.SHOOT_1, clock.getAsLong());
    }

    @Override public void loop() {
        if (running) {
            if (gamepad1.b || gamepad2.b) { stop(); requestOpModeStop(); return; }
            long now = clock.getAsLong();
            follower.update();
            color.update();
            if (RedAutoPlan.isDrive(step) || RedAutoPlan.isTurn(step)) {
                if (!follower.isBusy()) {
                    // A completed Pedro turn retains a powered hold until explicitly cancelled.
                    follower.breakFollowing();
                    advance(now);
                }
            } else if (RedAutoPlan.isShoot(step)) {
                updateShooting(now);
            } else if (elapsed(now, stepStarted) >= settings.flowerSettlingSeconds) {
                advance(now);
            }
        }
        showTelemetry();
    }

    private void enter(RedAutoPlan.Step next, long now) {
        step = next;
        target = plan.getTarget(step);
        stepStarted = now;
        activePath = null;
        feeding = false;
        status = "RUNNING";
        if (step == RedAutoPlan.Step.INTAKE_1 || step == RedAutoPlan.Step.INTAKE_2) intake.intake();
        else intake.stop();
        transfer.stop();
        shooter.stop();
        feedStatus = "STOPPED";

        if (RedAutoPlan.isShoot(step)) {
            shooter.setAutoMode();
            feedingBall = BallColorSensor.BallType.UNKNOWN;
            feedStatus = "WAITING FOR SPEED / BALL / SERVO";
        } else if (RedAutoPlan.isDrive(step)) {
            Pose from = plan.getTarget(RedAutoPlan.Step.values()[step.ordinal() - 1]);
            activePath = new Path(new BezierLine(from, target), new PathConstraints(
                    0.995, settings.stoppedSpeed, settings.positionTolerance, settings.headingTolerance,
                    settings.endpointTimeoutMs, Constants.pathConstraints.getBrakingStrength(),
                    Constants.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(), Constants.pathConstraints.getBrakingStart()));
            activePath.setConstantHeadingInterpolation(target.getHeading());
            follower.setMaxPower(Math.min(settings.pathPower, Constants.mecanumMaxPower));
            follower.followPath(activePath, false);
        } else if (RedAutoPlan.isTurn(step)) {
            follower.setMaxPower(Math.min(settings.pathPower, Constants.mecanumMaxPower));
            double rotation = RedAutoPlan.turnRadians(step) > 0.0
                    ? positiveAngle(target.getHeading() - follower.getPose().getHeading())
                    : -positiveAngle(follower.getPose().getHeading() - target.getHeading());
            follower.turn(rotation);
        } else if (step == RedAutoPlan.Step.FLOWER_DOWN) {
            flower.lower();
            flower2.setPosition(settings.flower2Lowered);
        }
    }

    private void updateShooting(long now) {
        if (feeding) {
            if (elapsed(now, feedStarted) >= settings.feedDurationSeconds) {
                intake.stop();
                transfer.stop();
                shooter.stop();
                feeding = false;
                advance(now);
            } else {
                feedStatus = String.format("FEEDING %.2f / %.2f s (%s)",
                        elapsed(now, feedStarted), settings.feedDurationSeconds, feedingBall);
            }
            return;
        }
        BallColorSensor.BallType detected = color.getBallType();
        if (detected == BallColorSensor.BallType.POLLEN) shooter.setTargetVelocity(POLLEN_TICKS_PER_SECOND);
        else if (detected == BallColorSensor.BallType.NECTAR) shooter.setTargetVelocity(NECTAR_TICKS_PER_SECOND);
        shooter.update(detected);
        if (shooter.isReadyToFeed()) {
            // Preserve the first identified ball's speed/adjustment throughout the continuous burst.
            feedingBall = detected;
            shooter.setSelectionMode(Shooter.SelectionMode.MANUAL);
            intake.intake();
            transfer.feed();
            feeding = true;
            feedStarted = now;
            feedStatus = "FEEDING - both motors ON for 4 seconds";
        } else feedStatus = "WAITING FOR SPEED / BALL / SERVO";
    }

    private void advance(long now) {
        if (step == RedAutoPlan.Step.SHOOT_3) {
            stop();
            status = "COMPLETE - three feed bursts ended; ball count not measured";
            requestOpModeStop();
        } else enter(RedAutoPlan.Step.values()[step.ordinal() + 1], now);
    }

    @Override public void stop() {
        running = feeding = false;
        status = "STOPPED";
        feedStatus = "STOPPED";
        // STOP remains independent across devices; one failed write must not skip other motors.
        stopDevice("Pedro", () -> { if (follower != null) follower.breakFollowing(); });
        stopDevice("drivetrain", () -> { if (driveTrain != null) driveTrain.stop(); });
        stopDevice("intake", () -> { if (intake != null) intake.stop(); });
        stopDevice("transfer", () -> { if (transfer != null) transfer.stop(); });
        stopDevice("flywheels", () -> { if (shooter != null) shooter.stop(); });
    }

    private void stopDevice(String name, Runnable action) {
        try { action.run(); }
        catch (RuntimeException e) { RobotLog.ee("RedAuto", e, "STOP failed: " + name); }
    }

    private void showTelemetry() {
        telemetry.addData("Auto", status);
        telemetry.addData("State 1-16", step == null ? "NOT STARTED" : (step.ordinal() + 1) + "/16 " + step);
        Pose pose = follower.getPose();
        telemetry.addData("Pedro X/Y/heading", poseText(pose));
        telemetry.addData("Target X/Y/heading", target == null ? "NOT STARTED" : poseText(target));
        telemetry.addData("Heading error (degrees)", target == null ? 0.0 : Math.toDegrees(Math.atan2(
                Math.sin(target.getHeading() - pose.getHeading()), Math.cos(target.getHeading() - pose.getHeading()))));
        telemetry.addData("Follower", follower.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("Path progress t", activePath == null ? "N/A (turn/action)" : activePath.getClosestPointTValue());
        telemetry.addData("Flywheel A/B ticks/s", "%.1f / %.1f", shooter.getVelocityA(), shooter.getVelocityB());
        telemetry.addData("Flywheel target / at speed", "%.0f / %s", shooter.getTargetVelocity(), shooter.isAtSpeed());
        telemetry.addData("Intake / transfer", intake.getState() + " / " + transfer.getState());
        telemetry.addData("Feed", feedStatus);
        telemetry.addData("Flower", flower.getState());
        telemetry.addData("Color", color.getBallType() + " / " + color.getStatus());
        telemetry.addData("Color calibration", color.getCalibrationStatus());
        telemetry.addLine("B on either gamepad stops autonomous. Feed duration is not a ball count.");
        telemetry.update();
    }

    private static double elapsed(long now, long start) { return (now - start) / 1_000_000_000.0; }
    private static double positiveAngle(double angle) { return (angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI); }
    private static String poseText(Pose pose) { return String.format("%.2f / %.2f / %.1f deg",
            pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())); }
}
