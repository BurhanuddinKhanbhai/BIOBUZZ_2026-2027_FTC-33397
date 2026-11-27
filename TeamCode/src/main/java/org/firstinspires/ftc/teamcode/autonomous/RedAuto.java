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

import java.util.function.LongSupplier;

/** Seventeen robot-relative autonomous steps, including editable zero-distance placeholders. */
@Autonomous(name = "BIOBUZZ - Red Auto", group = "Competition")
public class RedAuto extends OpMode {
    public static final class Settings {
        public double positionTolerance = 1.0;
        public double headingTolerance = Math.toRadians(5.0);
        public double stoppedSpeed = 1.0;
        public double endpointTimeoutMs = 11000.0;

        // Speeds when collecting balls
        public double pickupIntakePower = 0.60;
        public double pickupTransferPower = 0.1;

        // Speeds when feeding balls into the shooter
        public double shootingIntakePower = 1.0;
        public double shootingTransferPower = 0.90;

        public double flywheelTolerance = 75.0;
        public double flywheelStableSeconds = 0.25;
        public double pollenPosition = 0.10;
        public double nectarPosition = 0.00;
        public double adjustmentSettlingSeconds = 0.05;
        public double flower1Raised = 0.3;
        public double flower1Lowered = 0.5;
        public double flower2Lowered = 0.5;
        public Servo.Direction flower2Direction = Servo.Direction.REVERSE;
    }


    private static final double SHOOT_1_SPEED = 1200.0;
    private static final double SHOOT_2_SPEED = 1000.0;
    private static final double SHOOT_3_SPEED = 1000.0;

    private final Settings settings;
    private final LongSupplier clock;
    private RedAutoPlan plan;
    private RedAutoPlan.Step step;
    private Follower follower;
    private DriveTrain driveTrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private FlowerMechanism flower;
    private Servo flower2;
    private Path activePath;
    private Pose target;
    private boolean running, feeding;
    private long stepStarted, feedStarted;
    private String status = "INITIALIZING";
    private String feedStatus = "STOPPED";

    public RedAuto() { this(new Settings(), System::nanoTime); }
    RedAuto(Settings settings, LongSupplier clock) {
        this.settings = settings;
        this.clock = clock;
    }

    @Override public void init() {
        plan = new RedAutoPlan();
        driveTrain = new DriveTrain(hardwareMap);

        intake = new Intake(
                hardwareMap,
                DcMotorSimple.Direction.REVERSE,
                settings.pickupIntakePower
        );

        transfer = new Transfer(
                hardwareMap,
                DcMotorSimple.Direction.REVERSE,
                settings.pickupTransferPower
        );

        shooter = new Shooter(hardwareMap, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                settings.flywheelTolerance, settings.flywheelStableSeconds,
                Servo.Direction.FORWARD, settings.adjustmentSettlingSeconds);
        shooter.configureAdjustmentPositions(settings.pollenPosition, settings.nectarPosition);
        shooter.setAutoMode();
        flower = new FlowerMechanism(hardwareMap);
        flower.configurePositions(settings.flower1Raised, settings.flower1Lowered);
        flower2 = hardwareMap.get(Servo.class, "flowerServo2");
        flower2.setDirection(settings.flower2Direction);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(plan.getStart());
        status = "READY";
        showTelemetry();
    }

    @Override public void init_loop() { showTelemetry(); }


    @Override public void start() {
        if (gamepad1.b || gamepad2.b) {
            stop();
            requestOpModeStop();
            return;
        }

        follower.setStartingPose(plan.getStart());

        // Start the shooter once and maintain its speed throughout autonomous.
        shooter.setPollenMode();
        shooter.setTargetVelocity(SHOOT_1_SPEED);
        running = true;
        enter(RedAutoPlan.Step.CLEAR_WALL_FORWARD, clock.getAsLong());
    }


    @Override public void loop() {
        if (running) {
            if (gamepad1.b || gamepad2.b) { stop(); requestOpModeStop(); return; }
            long now = clock.getAsLong();
            follower.update();

            if (RedAutoPlan.isTurn(step)) {
                double error = Math.atan2(
                        Math.sin(target.getHeading() - follower.getPose().getHeading()),
                        Math.cos(target.getHeading() - follower.getPose().getHeading()));
                if (Math.abs(error) <= Math.toRadians(step.getFinishTolerance())) {
                    follower.breakFollowing();
                    driveTrain.stop();
                    advance(now);
                }

            } else if (RedAutoPlan.isDrive(step)) {

                if (step == RedAutoPlan.Step.INTAKE_1) {

                    if (activePath != null) {
                        double distance = Math.hypot(
                                target.getX() - follower.getPose().getX(),
                                target.getY() - follower.getPose().getY()
                        );

                        if (distance <= step.getFinishTolerance()) {
                            follower.breakFollowing();
                            driveTrain.stop();
                            activePath = null;

                            // Start the waiting timer after reaching the intake.
                            stepStarted = now;
                        }
                    }

                    // Keep intaking while stationary for the configured duration.
                    if (activePath == null &&
                            elapsed(now, stepStarted) >= step.getDurationSeconds()) {
                        advance(now);
                    }

                } else if (step == RedAutoPlan.Step.INTAKE_FLOWER) {


                    boolean intakeTimeComplete =
                            elapsed(now, stepStarted) >= step.getDurationSeconds();


                    if (intakeTimeComplete) {
                        intake.stop();
                        transfer.stop();
                    }


                    if (activePath != null) {
                        double distance = Math.hypot(
                                target.getX() - follower.getPose().getX(),
                                target.getY() - follower.getPose().getY()
                        );

                        if (distance <= step.getFinishTolerance()) {
                            follower.breakFollowing();
                            driveTrain.stop();
                            activePath = null;
                        }
                    }

                    if (activePath == null && intakeTimeComplete) {
                        advance(now);
                    }

                } else if (activePath == null) {

                    advance(now);

                } else {

                    double distance = Math.hypot(
                            target.getX() - follower.getPose().getX(),
                            target.getY() - follower.getPose().getY()
                    );

                    if (distance <= step.getFinishTolerance()) {
                        follower.breakFollowing();
                        advance(now);
                    }
                }


            } else if (RedAutoPlan.isShoot(step)) {
                updateShooting(now);
            } else if (elapsed(now, stepStarted) >= step.getDurationSeconds()) {
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


        boolean intakeStep = step == RedAutoPlan.Step.INTAKE_1
                || (step == RedAutoPlan.Step.INTAKE_FLOWER
                && !plan.isZeroDistanceDrive(step));

        if (intakeStep) {
            // Pickup speeds
            intake.setPower(settings.pickupIntakePower);
            transfer.setPower(settings.pickupTransferPower);
        } else {
            intake.stop();
            transfer.stop();
        }

        feedStatus = "STOPPED";


        if (RedAutoPlan.isShoot(step)) {
            feedStatus = "WAITING FOR FLYWHEEL / SERVO";
        } else if (RedAutoPlan.isDrive(step)) {
            // Avoid constructing or following a zero-length Pedro path.
            if (plan.isZeroDistanceDrive(step)) return;

            Pose from = follower.getPose();
            // Pedro's path constraint stays at the original global setting (1 inch),
            // except for clearance moves that require a tighter finish tolerance.
            double pathTolerance = Math.min(settings.positionTolerance,
                    step.getFinishTolerance());
            activePath = new Path(new BezierLine(from, target), new PathConstraints(
                    0.995, settings.stoppedSpeed, pathTolerance,
                    settings.headingTolerance, settings.endpointTimeoutMs,
                    Constants.pathConstraints.getBrakingStrength(),
                    Constants.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(),
                    Constants.pathConstraints.getBrakingStart()));
            activePath.setConstantHeadingInterpolation(target.getHeading());

            follower.setMaxPower(Math.min(step.getMaxPower(), Constants.mecanumMaxPower));
            follower.followPath(activePath, false);
        } else if (RedAutoPlan.isTurn(step)) {
            follower.setMaxPower(Math.min(step.getMaxPower(), Constants.mecanumMaxPower));
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
        double duration = step.getDurationSeconds();
        if (feeding) {
            if (elapsed(now, feedStarted) >= duration) {
                intake.stop();
                transfer.stop();
                feeding = false;
                advance(now);
            } else {
                feedStatus = String.format("FEEDING %.2f / %.2f s (POLLEN)",
                        elapsed(now, feedStarted), duration);
            }
            return;
        }

        if (shooter.isReadyToFeed()) {
            intake.setPower(settings.shootingIntakePower);
            transfer.setPower(settings.shootingTransferPower);

            feeding = true;
            feedStarted = now;
            feedStatus = "FEEDING POLLEN";
        } else {
            feedStatus = "WAITING FOR FLYWHEEL / SERVO";
        }
    }



    private void advance(long now) {
        if (step == RedAutoPlan.Step.PARK_BACKWARD) {
            stop();
            status = "COMPLETE - parked";
            requestOpModeStop();
        } else {

            // Adjust flywheel velocity between shooting sequences.
            // The flywheels remain running throughout autonomous.
            if (step == RedAutoPlan.Step.SHOOT_1) {
                shooter.setTargetVelocity(SHOOT_2_SPEED);
            } else if (step == RedAutoPlan.Step.SHOOT_2) {
                shooter.setTargetVelocity(SHOOT_3_SPEED);
            }

            enter(RedAutoPlan.Step.values()[step.ordinal() + 1], now);
        }
    }



    @Override public void stop() {
        running = feeding = false;
        status = "STOPPED";
        feedStatus = "STOPPED";
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
        telemetry.addData("State 1-17", step == null ? "NOT STARTED" : (step.ordinal() + 1) + "/17 " + step);
        Pose pose = follower.getPose();
        telemetry.addData("Pedro X/Y/heading", poseText(pose));
        telemetry.addData("Target X/Y/heading", target == null ? "NOT STARTED" : poseText(target));
        telemetry.addData("Heading error (degrees)", target == null ? 0.0 : Math.toDegrees(Math.atan2(
                Math.sin(target.getHeading() - pose.getHeading()),
                Math.cos(target.getHeading() - pose.getHeading()))));
        telemetry.addData("Follower", follower.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("Path progress t", activePath == null ? "N/A (turn/action/placeholder)"
                : activePath.getClosestPointTValue());
        telemetry.addData("Flywheel A/B ticks/s", "%.1f / %.1f", shooter.getVelocityA(), shooter.getVelocityB());
        telemetry.addData("Flywheel target / at speed", "%.0f / %s", shooter.getTargetVelocity(), shooter.isAtSpeed());
        telemetry.addData("Intake / transfer", intake.getState() + " / " + transfer.getState());
        telemetry.addData("Feed", feedStatus);
        telemetry.addData("Flower", flower.getState());
        telemetry.addLine("B on either gamepad stops autonomous; zero-distance placeholders are skipped.");
        telemetry.update();
    }

    private static double elapsed(long now, long start) { return (now - start) / 1_000_000_000.0; }
    private static double positiveAngle(double angle) {
        return (angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
    }
    private static String poseText(Pose pose) {
        return String.format("%.2f / %.2f / %.1f deg",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
