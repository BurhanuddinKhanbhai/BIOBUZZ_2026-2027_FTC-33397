package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
// Powered tuning requires a configured and verified mecanum drivetrain and three-wheel localizer.
@Disabled
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
    public static Follower follower;

    // Software watchdog, not a calibration value. Shorten for the available test space.
    public static double POWERED_TIMEOUT_SECONDS = 30.0;
    private static boolean motionStopped;
    static SafeTuning activeTest;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", () -> new SafeTuning(new LocalizationTest(), true));
                l.add("Offsets Tuner", () -> new SafeTuning(new OffsetsTuner(), false));
                l.add("Forward Tuner", () -> new SafeTuning(new ForwardTuner(), false));
                l.add("Lateral Tuner", () -> new SafeTuning(new LateralTuner(), false));
                l.add("Turn Tuner", () -> new SafeTuning(new TurnTuner(), false));
            });
            s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", () -> new SafeTuning(new ForwardVelocityTuner(), true));
                a.add("Lateral Velocity Tuner", () -> new SafeTuning(new LateralVelocityTuner(), true));
                a.add("Forward Zero Power Acceleration Tuner", () -> new SafeTuning(new ForwardZeroPowerAccelerationTuner(), true));
                a.add("Lateral Zero Power Acceleration Tuner", () -> new SafeTuning(new LateralZeroPowerAccelerationTuner(), true));
                a.add("Predictive Braking Tuner", () -> new SafeTuning(new PredictiveBrakingTuner(), true));
            });
            s.folder("Manual", p -> {
                p.add("Translational Tuner", () -> new SafeTuning(new TranslationalTuner(), true));
                p.add("Heading Tuner", () -> new SafeTuning(new HeadingTuner(), true));
                p.add("Drive Tuner", () -> new SafeTuning(new DriveTuner(), true));
                p.add("Centripetal Tuner", () -> new SafeTuning(new CentripetalTuner(), true));
            });
            s.folder("Tests", p -> {
                p.add("Line", () -> new SafeTuning(new Line(), true));
                p.add("Triangle", () -> new SafeTuning(new Triangle(), true));
                p.add("Circle", () -> new SafeTuning(new Circle(), true));
            });
            s.folder("Swerve", p-> {
                p.add("Analog Min / Max Tuner", () -> new SafeTuning(new AnalogMinMaxTuner(), false));
                p.add("Swerve Offsets Test", () -> new SafeTuning(new SwerveOffsetsTest(), true));
                p.add("Swerve Turn Test", () -> new SafeTuning(new SwerveTurnTest(), true));
            });
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

    /** Cancel once per motion; repeatedly restarting TeleOp would grow Pedro's sample buffers. */
    public static void stopRobot() {
        if (motionStopped || follower == null) {
            return;
        }
        motionStopped = true;
        RuntimeException failure = null;
        try {
            follower.setTeleOpDrive(0, 0, 0, true);
        } catch (RuntimeException e) {
            failure = e;
            RobotLog.ee("PedroTuning", e, "Failed to clear drive request");
        }
        try {
            // Pedro 2.1.2 cancels motion and requests zero drivetrain output here.
            follower.breakFollowing();
        } catch (RuntimeException e) {
            failure = e;
            RobotLog.ee("PedroTuning", e, "Failed to cancel follower motion");
            // Pedro's motor loop stops at the first exception. Bypass its power cache and
            // independently attempt every motor once; the failure still latches the test off.
            try {
                if (follower.getDrivetrain() instanceof Mecanum) {
                    for (DcMotorEx motor : ((Mecanum) follower.getDrivetrain()).getMotors()) {
                        try {
                            motor.setPower(0.0);
                        } catch (RuntimeException motorFailure) {
                            RobotLog.ee("PedroTuning", motorFailure, "Mecanum motor zero-power fallback failed");
                        }
                    }
                }
            } catch (RuntimeException fallbackFailure) {
                RobotLog.ee("PedroTuning", fallbackFailure, "Could not access mecanum motors for shutdown");
            }
        }
        if (failure != null) {
            throw new IllegalStateException("Follower stop failed; inspect hardware and RobotLog", failure);
        }
    }

    static void beginMotion() {
        motionStopped = false;
    }

    static void updateFollower() {
        // breakFollowing retains path references in 2.1.2: do not update after stopping.
        if (!motionStopped) {
            requireFeedback();
            follower.update();
            requireFeedback();
        }
    }

    static void requireFeedback() {
        if (follower == null || !TuningMath.validPose(follower.getPose())
                || !TuningMath.validVelocity(follower.getVelocity())
                || !TuningMath.isFinite(follower.getTotalHeading())) {
            throw new IllegalStateException("Missing or nonfinite localization feedback");
        }
    }

    static void requireTestDistance(double distance) {
        if (!TuningMath.isFinite(distance) || distance == 0.0) {
            throw new IllegalArgumentException("Test distance/radius must be finite and nonzero");
        }
    }

    static void completeTest() {
        stopRobot();
        if (activeTest != null) {
            activeTest.completed = true;
        }
    }

    static void failTest(String reason) {
        if (activeTest != null) {
            activeTest.halt(reason);
        } else {
            stopRobot();
            if (telemetryM != null) telemetryM.debug("Test failed: " + reason);
        }
    }
}

/** Shared watchdog for selector children, which do not receive SDK stop-request services. */
class SafeTuning extends OpMode {
    private final OpMode child;
    private final boolean powered;
    private long startedNanos;
    private double timeoutSeconds;
    private boolean started;
    private boolean halted;
    boolean completed;
    private String failure;

    SafeTuning(OpMode child, boolean powered) {
        this.child = child;
        this.powered = powered;
    }

    @Override public void init() {
        Tuning.activeTest = this;
        Tuning.beginMotion();
        child.hardwareMap = hardwareMap;
        child.gamepad1 = gamepad1;
        child.gamepad2 = gamepad2;
        child.telemetry = telemetry;
        try {
            child.init();
        } catch (RuntimeException e) {
            fail(e);
        }
    }

    @Override public void init_loop() {
        if (!halted) {
            try { child.init_loop(); } catch (RuntimeException e) { fail(e); }
        }
        report();
    }

    @Override public void start() {
        if (halted || started) return;
        started = true;
        timeoutSeconds = Tuning.POWERED_TIMEOUT_SECONDS;
        startedNanos = System.nanoTime();
        try {
            if (powered && (!TuningMath.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0)) {
                throw new IllegalArgumentException("Powered timeout must be finite and positive");
            }
            if (gamepad1.b || gamepad2.b) { halt("Operator abort"); return; }
            if (powered) Tuning.requireFeedback();
            child.start();
            if (powered && !completed) Tuning.requireFeedback();
        } catch (RuntimeException e) {
            fail(e);
        }
    }

    @Override public void loop() {
        if (!started || halted) { report(); return; }
        try {
            if (gamepad1.b || gamepad2.b) { halt("Operator abort"); report(); return; }
            if (powered && !completed) {
                if ((System.nanoTime() - startedNanos) / 1e9 >= timeoutSeconds) {
                    halt("Powered test timed out; stop and reselect to retry"); report(); return;
                }
                Tuning.requireFeedback();
            }
            child.loop();
            if (powered && !completed && !halted) Tuning.requireFeedback();
        } catch (RuntimeException e) {
            fail(e);
        }
        report();
    }

    void halt(String reason) {
        if (!halted) {
            halted = true;
            failure = reason;
            try {
                Tuning.stopRobot();
            } catch (RuntimeException e) {
                failure += "; " + e.getMessage();
                RobotLog.ee("PedroTuning", e, "Shutdown failed; test remains halted");
            }
        }
    }

    private void fail(RuntimeException e) {
        RobotLog.ee("PedroTuning", e, "Tuning operation failed");
        halt(e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage());
    }

    private void report() {
        if (halted) {
            telemetry.addData("Tuning stopped", failure);
            telemetry.addData("Retry", "Stop and reselect the test; motion will not resume automatically");
            telemetry.update();
        }
    }

    @Override public void stop() {
        halt("OpMode stopped");
        // These existing children have no shutdown work; never dispatch another motion update.
    }
}

/**
 * This is the LocalizationTest OpMode. This is basically just a simple drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Kabir Goyal
 * @version 1.0, 5/6/2024
 */
class LocalizationTest extends OpMode {
    boolean debugStringEnabled = false;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }


        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic drive on gamepad 1.");
        telemetryM.debug("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Tuning.updateFollower();
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        Tuning.updateFollower();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        if (debugStringEnabled) {
            telemetryM.debug("Drivetrain Debug String:\n" +
                    follower.getDrivetrain().debugString());
        }
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class ForwardTuner extends OpMode {
    public static double DISTANCE = 48;
    private Pose startingPose;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        Tuning.updateFollower();
        drawCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.updateFollower();
        startingPose = follower.getPose();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();

        double displacement = TuningMath.displacement(startingPose, follower.getPose(), false);
        double multiplier = DISTANCE > 0 ? TuningMath.multiplier(DISTANCE, displacement,
                follower.getPoseTracker().getLocalizer().getForwardMultiplier()) : Double.NaN;
        telemetryM.debug("Forward Distance Moved: " + displacement);
        telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (TuningMath.isFinite(multiplier) ? multiplier
                : "Insufficient forward displacement or invalid measurement/configuration"));
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
class LateralTuner extends OpMode {
    public static double DISTANCE = 48;
    private Pose startingPose;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        Tuning.updateFollower();
        drawCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.updateFollower();
        startingPose = follower.getPose();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();

        // Pedro's positive lateral axis points left; this manual test requests rightward travel.
        double displacement = -TuningMath.displacement(startingPose, follower.getPose(), true);
        double multiplier = DISTANCE > 0 ? TuningMath.multiplier(DISTANCE, displacement,
                follower.getPoseTracker().getLocalizer().getLateralMultiplier()) : Double.NaN;
        telemetryM.debug("Rightward Distance Moved: " + displacement);
        telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (TuningMath.isFinite(multiplier) ? multiplier
                : "Insufficient rightward displacement or invalid measurement/configuration"));
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class TurnTuner extends OpMode {
    public static double ANGLE = 2 * Math.PI;
    private double startingHeading = Double.NaN;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        Tuning.updateFollower();
        drawCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);

        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.updateFollower();
        startingHeading = follower.getTotalHeading();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();

        double displacement = follower.getTotalHeading() - startingHeading;
        double multiplier = TuningMath.multiplier(ANGLE, displacement,
                follower.getPoseTracker().getLocalizer().getTurningMultiplier());
        telemetryM.debug("Angle Moved: " + displacement);
        telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetryM.debug("Multiplier: " + (TuningMath.isFinite(multiplier) ? multiplier
                : "Insufficient rotation in the requested direction or invalid measurement/configuration"));
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class ForwardVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;
    private int sampleCount;
    private double distance;
    private Pose startPose;
    private boolean end;
    private double average = Double.NaN;

    @Override public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override public void init_loop() {
        telemetryM.debug("Forward velocity test: full power over " + DISTANCE + " inches; B aborts.");
        telemetryM.debug("Powered runtime is bounded by Tuning.POWERED_TIMEOUT_SECONDS.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override public void start() {
        sampleCount = TuningMath.sampleCount(RECORD_NUMBER);
        distance = DISTANCE;
        if (!TuningMath.isFinite(distance) || distance <= 0.0) {
            throw new IllegalArgumentException("Test distance must be finite and positive");
        }
        velocities.clear();
        end = false;
        average = Double.NaN;
        Pose pose = follower.getPose();
        startPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        follower.startTeleopDrive(true);
    }

    @Override public void loop() {
        if (gamepad1.b || gamepad2.b) { Tuning.failTest("Operator abort"); return; }
        if (!end) {
            Tuning.updateFollower();
            double moved = TuningMath.displacement(startPose, follower.getPose(), false);
            if (!TuningMath.isFinite(moved) || moved < 0.0) {
                throw new IllegalStateException("Invalid displacement or movement opposite commanded direction");
            }
            if (moved >= distance) {
                end = true;
                Tuning.completeTest();
                average = TuningMath.mean(velocities, sampleCount);
                if (average <= 0.0) throw new IllegalStateException("No positive measured velocity");
            } else {
                double heading = startPose.getHeading();
                double velocity = follower.getVelocity().dot(new Vector(1.0, heading));
                if (!TuningMath.isFinite(velocity) || velocity < 0.0) {
                    throw new IllegalStateException("Invalid velocity or velocity opposite commanded direction");
                }
                velocities.add(velocity);
                if (velocities.size() > sampleCount) velocities.remove(0);
                follower.setTeleOpDrive(1, 0, 0, true);
            }
            drawCurrentAndHistory();
        }
        if (end) {
            telemetryM.debug("Forward Velocity: " + average);
            telemetryM.debug("Press A to apply this validated velocity temporarily; restart to rerun.");
            if (gamepad1.aWasPressed() && TuningMath.isFinite(average) && average > 0.0) {
                follower.setXVelocity(average);
                changes.add("Forward Velocity: " + average);
            }
            telemetryM.update(telemetry);
        }
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class LateralVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;
    private int sampleCount;
    private double distance;
    private Pose startPose;
    private boolean end;
    private double average = Double.NaN;

    @Override public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override public void init_loop() {
        telemetryM.debug("Lateral velocity test: full power over " + DISTANCE + " inches; B aborts.");
        telemetryM.debug("Powered runtime is bounded by Tuning.POWERED_TIMEOUT_SECONDS.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override public void start() {
        sampleCount = TuningMath.sampleCount(RECORD_NUMBER);
        distance = DISTANCE;
        if (!TuningMath.isFinite(distance) || distance <= 0.0) {
            throw new IllegalArgumentException("Test distance must be finite and positive");
        }
        velocities.clear();
        end = false;
        average = Double.NaN;
        Pose pose = follower.getPose();
        startPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        follower.startTeleopDrive(true);
    }

    @Override public void loop() {
        if (gamepad1.b || gamepad2.b) { Tuning.failTest("Operator abort"); return; }
        if (!end) {
            Tuning.updateFollower();
            double moved = TuningMath.displacement(startPose, follower.getPose(), true);
            if (!TuningMath.isFinite(moved) || moved < 0.0) {
                throw new IllegalStateException("Invalid displacement or movement opposite commanded direction");
            }
            if (moved >= distance) {
                end = true;
                Tuning.completeTest();
                average = TuningMath.mean(velocities, sampleCount);
                if (average <= 0.0) throw new IllegalStateException("No positive measured velocity");
            } else {
                double heading = startPose.getHeading() + Math.PI / 2;
                double velocity = follower.getVelocity().dot(new Vector(1.0, heading));
                if (!TuningMath.isFinite(velocity) || velocity < 0.0) {
                    throw new IllegalStateException("Invalid velocity or velocity opposite commanded direction");
                }
                velocities.add(velocity);
                if (velocities.size() > sampleCount) velocities.remove(0);
                follower.setTeleOpDrive(0, 1, 0, true);
            }
            drawCurrentAndHistory();
        }
        if (end) {
            telemetryM.debug("Lateral Velocity: " + average);
            telemetryM.debug("Press A to apply this validated velocity temporarily; restart to rerun.");
            if (gamepad1.aWasPressed() && TuningMath.isFinite(average) && average > 0.0) {
                follower.setYVelocity(average);
                changes.add("Lateral Velocity: " + average);
            }
            telemetryM.update(telemetry);
        }
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
class ForwardZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;
    private double targetVelocity;
    private double stopVelocity;
    private double heading;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;
    private double average = Double.NaN;

    @Override public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override public void init_loop() {
        telemetryM.debug("Forward deceleration test: accelerate to " + VELOCITY + " inches/s, then coast.");
        telemetryM.debug("B aborts; Tuning.POWERED_TIMEOUT_SECONDS bounds acceleration and coasting.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override public void start() {
        targetVelocity = VELOCITY;
        stopVelocity = follower.getConstraints().getVelocityConstraint();
        if (!TuningMath.isFinite(targetVelocity) || targetVelocity <= 0.0
                || !TuningMath.isFinite(stopVelocity) || stopVelocity <= 0.0 || stopVelocity >= targetVelocity) {
            throw new IllegalArgumentException("Require finite 0 < stopping threshold < target velocity");
        }
        accelerations.clear();
        stopping = end = false;
        average = Double.NaN;
        heading = follower.getPose().getHeading();
        follower.startTeleopDrive(false);
        follower.setTeleOpDrive(1, 0, 0, true);
    }

    @Override public void loop() {
        if (gamepad1.b || gamepad2.b) { Tuning.failTest("Operator abort"); return; }
        if (!end) {
            Tuning.updateFollower();
            double velocity = follower.getVelocity().dot(new Vector(1.0, heading));
            if (!TuningMath.isFinite(velocity) || velocity < 0.0) {
                throw new IllegalStateException("Invalid velocity or motion opposite commanded direction");
            }
            long now = System.nanoTime();
            if (!stopping) {
                if (velocity >= targetVelocity) {
                    previousVelocity = velocity;
                    previousTimeNano = now;
                    stopping = true;
                    // Remain in the existing FLOAT TeleOp mode while measuring natural deceleration.
                    follower.setTeleOpDrive(0, 0, 0, true);
                    Tuning.updateFollower();
                }
            } else {
                double dt = (now - previousTimeNano) / 1e9;
                double acceleration = (velocity - previousVelocity) / dt;
                if (dt <= 0.0 || !TuningMath.isFinite(acceleration)) {
                    throw new IllegalStateException("Invalid deceleration sample interval/value");
                }
                accelerations.add(acceleration);
                previousVelocity = velocity;
                previousTimeNano = now;
                if (velocity <= stopVelocity) {
                    end = true;
                    Tuning.completeTest();
                    average = TuningMath.mean(accelerations, 2);
                    if (average >= 0.0) throw new IllegalStateException("Samples do not show deceleration");
                }
            }
            drawCurrentAndHistory();
        }
        if (end) {
            telemetryM.debug("Forward Zero Power Acceleration: " + average);
            telemetryM.debug("Press A to apply this validated deceleration temporarily; restart to rerun.");
            if (gamepad1.aWasPressed() && TuningMath.isFinite(average) && average < 0.0) {
                follower.getConstants().setForwardZeroPowerAcceleration(average);
                changes.add("Forward Zero Power Acceleration: " + average);
            }
            telemetryM.update(telemetry);
        }
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the right until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class LateralZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;
    private double targetVelocity;
    private double stopVelocity;
    private double heading;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;
    private double average = Double.NaN;

    @Override public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override public void init_loop() {
        telemetryM.debug("Lateral deceleration test: accelerate to " + VELOCITY + " inches/s, then coast.");
        telemetryM.debug("B aborts; Tuning.POWERED_TIMEOUT_SECONDS bounds acceleration and coasting.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override public void start() {
        targetVelocity = VELOCITY;
        stopVelocity = follower.getConstraints().getVelocityConstraint();
        if (!TuningMath.isFinite(targetVelocity) || targetVelocity <= 0.0
                || !TuningMath.isFinite(stopVelocity) || stopVelocity <= 0.0 || stopVelocity >= targetVelocity) {
            throw new IllegalArgumentException("Require finite 0 < stopping threshold < target velocity");
        }
        accelerations.clear();
        stopping = end = false;
        average = Double.NaN;
        heading = follower.getPose().getHeading() + Math.PI / 2;
        follower.startTeleopDrive(false);
        follower.setTeleOpDrive(0, 1, 0, true);
    }

    @Override public void loop() {
        if (gamepad1.b || gamepad2.b) { Tuning.failTest("Operator abort"); return; }
        if (!end) {
            Tuning.updateFollower();
            double velocity = follower.getVelocity().dot(new Vector(1.0, heading));
            if (!TuningMath.isFinite(velocity) || velocity < 0.0) {
                throw new IllegalStateException("Invalid velocity or motion opposite commanded direction");
            }
            long now = System.nanoTime();
            if (!stopping) {
                if (velocity >= targetVelocity) {
                    previousVelocity = velocity;
                    previousTimeNano = now;
                    stopping = true;
                    // Remain in the existing FLOAT TeleOp mode while measuring natural deceleration.
                    follower.setTeleOpDrive(0, 0, 0, true);
                    Tuning.updateFollower();
                }
            } else {
                double dt = (now - previousTimeNano) / 1e9;
                double acceleration = (velocity - previousVelocity) / dt;
                if (dt <= 0.0 || !TuningMath.isFinite(acceleration)) {
                    throw new IllegalStateException("Invalid deceleration sample interval/value");
                }
                accelerations.add(acceleration);
                previousVelocity = velocity;
                previousTimeNano = now;
                if (velocity <= stopVelocity) {
                    end = true;
                    Tuning.completeTest();
                    average = TuningMath.mean(accelerations, 2);
                    if (average >= 0.0) throw new IllegalStateException("Samples do not show deceleration");
                }
            }
            drawCurrentAndHistory();
        }
        if (end) {
            telemetryM.debug("Lateral Zero Power Acceleration: " + average);
            telemetryM.debug("Press A to apply this validated deceleration temporarily; restart to rerun.");
            if (gamepad1.aWasPressed() && TuningMath.isFinite(average) && average < 0.0) {
                follower.getConstants().setLateralZeroPowerAcceleration(average);
                changes.add("Lateral Zero Power Acceleration: " + average);
            }
            telemetryM.update(telemetry);
        }
    }
}

/**
 * This is the Predictive Braking Tuner. It runs the robot forward and backward at various power
 * levels, recording the robot’s velocity and position immediately before braking. The motors are
 * then set to a reverse power, which represents the fastest theoretical braking the robot
 * can achieve. Once the robot comes to a complete stop, the tuner measures the stopping distance.
 * Using the collected data, it generates a velocity-vs-stopping-distance graph and fits a
 * quadratic curve to model the braking behavior.
 *
 * @author Ashay Sarda - 19745 Turtle Walkers
 * @author Jacob Ophoven - 18535 Frozen Code
 * @version 1.0, 12/26/2025
 */
class PredictiveBrakingTuner extends OpMode {
    private static final double[] TEST_POWERS =
            {1, 1, 1, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    private static final double BRAKING_POWER = -0.2;
    private static final int DRIVE_TIME_MS = 1000;
    // A watchdog budget, not a measured braking time or a robot calibration.
    public static double BRAKE_TIMEOUT_SECONDS = 3.0;

    private enum State { START_MOVE, WAIT_DRIVE_TIME, WAIT_BRAKE_TIME, DONE }
    private State state = State.START_MOVE;
    private final ElapsedTime timer = new ElapsedTime();
    private int iteration;
    private Pose brakingStart;
    private double heading;
    private double measuredVelocity;
    private double brakeTimeout;
    private double[] coefficients;
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();

    @Override public void init() {}

    @Override public void init_loop() {
        telemetryM.debug("Predictive braking test: alternating forward/backward motion at existing test powers.");
        telemetryM.debug("B aborts. Both total runtime and each reverse-braking phase have watchdogs.");
        telemetryM.debug("Leave sufficient clear space. No coefficients are applied automatically.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override public void start() {
        brakeTimeout = BRAKE_TIMEOUT_SECONDS;
        if (!TuningMath.isFinite(brakeTimeout) || brakeTimeout <= 0.0) {
            throw new IllegalArgumentException("Braking timeout must be finite and positive");
        }
        iteration = 0;
        coefficients = null;
        velocityToBrakingDistance.clear();
        state = State.START_MOVE;
        timer.reset();
    }

    @Override public void loop() {
        if (gamepad1.b || gamepad2.b) { Tuning.failTest("Operator abort"); return; }
        if (state == State.DONE) {
            // No follower updates, fit recalculation, or repeated stopping after completion.
            telemetryM.debug("Tuning Complete; restart the test to run again.");
            telemetryM.debug("kLinearBraking", coefficients[0]);
            telemetryM.debug("kQuadraticFriction", coefficients[1]);
            telemetryM.update(telemetry);
            return;
        }
        Tuning.updateFollower();
        double direction = iteration % 2 == 0 ? 1.0 : -1.0;
        switch (state) {
            case START_MOVE:
                heading = follower.getPose().getHeading();
                Tuning.beginMotion();
                follower.startTeleopDrive(true);
                follower.setMaxPower(TEST_POWERS[iteration]);
                follower.setTeleOpDrive(direction, 0, 0, true);
                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            case WAIT_DRIVE_TIME:
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = direction * follower.getVelocity().dot(new Vector(1.0, heading));
                    if (!TuningMath.isFinite(measuredVelocity) || measuredVelocity <= 0.0) {
                        throw new IllegalStateException("No valid forward velocity for braking sample");
                    }
                    Pose pose = follower.getPose();
                    brakingStart = new Pose(pose.getX(), pose.getY(), pose.getHeading());
                    follower.setTeleOpDrive(BRAKING_POWER * direction, 0, 0, true);
                    timer.reset();
                    state = State.WAIT_BRAKE_TIME;
                }
                break;
            case WAIT_BRAKE_TIME:
                if (timer.seconds() >= brakeTimeout) {
                    Tuning.failTest("Reverse-braking phase timed out");
                    return;
                }
                double velocity = direction * follower.getVelocity().dot(new Vector(1.0, heading));
                if (!TuningMath.isFinite(velocity)) {
                    throw new IllegalStateException("Nonfinite braking velocity");
                }
                if (velocity <= 0.0) {
                    // Stop reverse torque immediately at the first observed zero crossing.
                    stopRobot();
                    Pose pose = follower.getPose();
                    double dx = pose.getX() - brakingStart.getX();
                    double dy = pose.getY() - brakingStart.getY();
                    double distance = direction * (dx * Math.cos(heading) + dy * Math.sin(heading));
                    if (!TuningMath.isFinite(distance) || distance <= 0.0) {
                        throw new IllegalStateException("Invalid or zero braking distance");
                    }
                    velocityToBrakingDistance.add(new double[]{measuredVelocity, distance});
                    iteration++;
                    if (iteration == TEST_POWERS.length) {
                        coefficients = TuningMath.brakingFit(velocityToBrakingDistance);
                        state = State.DONE;
                        Tuning.completeTest();
                    } else {
                        state = State.START_MOVE;
                    }
                }
                break;
            default:
                throw new IllegalStateException("Unexpected braking test state");
        }
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class TranslationalTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the translational PIDF(s)");
        telemetryM.debug("The robot will try to stay in place while you push it laterally.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.requireTestDistance(DISTANCE);
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error X", follower.errorCalculator.getTranslationalError().getXComponent());
        telemetryM.addData("Error Y", follower.errorCalculator.getTranslationalError().getYComponent());
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class HeadingTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the heading PIDF(s).");
        telemetryM.debug("The robot will try to stay at a constant heading while you try to turn it.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.requireTestDistance(DISTANCE);
        follower.deactivateAllPIDFs();
        follower.activateHeading();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error", follower.errorCalculator.getHeadingError());
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class DriveTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private PathChain forwards;
    private PathChain backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.");
        telemetryM.debug("The robot will go forward and backward continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.requireTestDistance(DISTANCE);
        follower.deactivateAllPIDFs();
        follower.activateDrive();

        forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)))
                .setConstantHeadingInterpolation(0)
                .build();

        backwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving forward?: " + forward);
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error", follower.errorCalculator.getDriveErrors()[1]);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class Line extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.requireTestDistance(DISTANCE);
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
class CentripetalTuner extends OpMode {
    public static double DISTANCE = 20;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
        telemetryM.debug("The robot will go continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        Tuning.requireTestDistance(DISTANCE);
        follower.activateAllPIDFs();
        forwards = new Path(new BezierCurve(new Pose(72,72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72)));
        backwards = new Path(new BezierCurve(new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(72,72)));

        backwards.setTangentHeadingInterpolation();
        backwards.reverseHeadingInterpolation();

        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving away from the origin along the curve?: " + forward);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
class Triangle extends OpMode {

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
    private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));

    private PathChain triangle;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    /** Creates the PathChain for the "triangle".*/
    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class Circle extends OpMode {
    public static double RADIUS = 10;
    private PathChain circle;

    public void start() {
        Tuning.requireTestDistance(RADIUS);
        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(RADIUS + 72, RADIUS + 72), new Pose(RADIUS + 72, (2 * RADIUS) + 72), new Pose(72, (2 * RADIUS) + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(-RADIUS + 72, RADIUS + 72), new Pose(-RADIUS + 72, 72), new Pose(72, 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .build();
        follower.followPath(circle);
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ");
        telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
        telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();
        drawCurrentAndHistory();

        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }
    }
}

/**
 * Tuning OpMode to get the min and max encoder values for swerve pods
 * @author Kabir Goyal
 */
class AnalogMinMaxTuner extends OpMode {
    //populate the below with your names for the servos and encoders
    public String[] encoderNames = {"leftFrontEncoder", "rightFrontEncoder", "leftBackEncoder", "rightBackEncoder"};
    public AnalogInput[] encoders = new AnalogInput[encoderNames.length];
    public double[] minVoltages = new double[encoderNames.length];
    public double[] maxVoltages = new double[encoderNames.length];

    public List<LynxModule> lynxModules; //js to improve loop times a bit yk

    public void start() {
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Press START. Then, Spin each pod slowly for 4 to 5 full rotations.\n" +
                "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry.");
        telemetryM.update(telemetry);
    }

    @Override
    public void init() {
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxModules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (int i = 0; i < encoders.length; i++)  {
            encoders[i] = hardwareMap.get(AnalogInput.class, encoderNames[i]);
            minVoltages[i] = 5; //bigger value than should ever be read
        }
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        for (LynxModule hub : lynxModules) {
            hub.clearBulkCache();
        }

        telemetryM.debug("Spin each pod slowly for 4 to 5 full rotations.\n" +
                "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry.\n\n");

        for (int i = 0; i < encoders.length; i++) {
            double currentVoltage = encoders[i].getVoltage();
            minVoltages[i] = Math.min(minVoltages[i], currentVoltage);
            maxVoltages[i] = Math.max(maxVoltages[i], currentVoltage);
            telemetryM.addData(encoderNames[i] + "min value:", minVoltages[i]);
            telemetryM.addData(encoderNames[i] + "max value:", maxVoltages[i]);
            telemetryM.addLine("");
        }

        telemetryM.update();
    }
}

/**
 * This is the SwerveOffsetsTest
 * You should use this to check how good your swerve angle offsets are and if your motor directions are correct
 * @author Kabir Goyal
 *
 */
class SwerveOffsetsTest extends OpMode {
    boolean debugStringEnabled = false;

    @Override
    public void init() {}

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }


        telemetryM.debug("This OpMode will run all four swerve pods in the direction they think is forward"
                + "\nensure your bot is not on the ground while running");
        telemetryM.debug("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Tuning.updateFollower();
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }

        follower.setTeleOpDrive(0.25, 0, 0, true);
        Tuning.updateFollower();

        if (debugStringEnabled) {
            telemetryM.debug("Drivetrain Debug String:\n" +
                    follower.getDrivetrain().debugString());
        }
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}

/**
 * This is the SwerveTurnTest
 * You should use this to check your encoder directions and x/y pod offsets
 * @author Kabir Goyal
 *
 */
class SwerveTurnTest extends OpMode {
    boolean debugStringEnabled = false;

    @Override
    public void init() {}

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }


        telemetryM.debug("This OpMode will run all four swerve pods in their turning direction (perpendicular to the center of the robot) "
                + "\nrun this once off the ground to check servo directions and motor directions before testing on the ground");
        telemetryM.debug("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetryM.update(telemetry);
        Tuning.updateFollower();
        drawCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Tuning.updateFollower();
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }

        follower.setTeleOpDrive(0, 0, 0.25, true);
        Tuning.updateFollower();

        if (debugStringEnabled) {
            telemetryM.debug("Drivetrain Debug String:\n" +
                    follower.getDrivetrain().debugString());
        }
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}

/**
 * This is the OffsetsTuner OpMode. This tracks the movement of the robot as it turns 180 degrees,
 * and calculates what the robot's strafeX and forwardY offsets should be. Ensure that your strafeX and forwardY offsets
 * are set to 0 before running this OpMode. After running, input the displayed offsets into your localizer constants.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @author Baron Henderson
 */
class OffsetsTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        Tuning.updateFollower();
        drawCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Prerequisite: Make sure both your offsets are set to 0 in your localizer constants.");
        telemetryM.debug("Turn your robot " + Math.PI + " radians. Your offsets in inches will be shown on the telemetry.");
        telemetryM.update(telemetry);

        drawCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated offsets and draws the robot.
     */
    @Override
    public void loop() {
        Tuning.updateFollower();

        telemetryM.debug("Total Angle: " + follower.getTotalHeading());

        telemetryM.debug("The following values are the offsets in inches that should be applied to your localizer.");
        telemetryM.debug("strafeX: " + ((72.0-follower.getPose().getX()) / 2.0));
        telemetryM.debug("forwardY: " + ((72.0-follower.getPose().getY()) / 2.0));
        telemetryM.update(telemetry);

        drawCurrentAndHistory();
    }
}


/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        if (path == null) return;
        double[][] points = path.getPanelsDrawingPoints();
        if (points == null || points.length != 2 || points[0] == null || points[1] == null
                || points[0].length != points[1].length) return;
        panelsField.setStyle(style);
        // Pedro 2.1.2 stores [x coordinates][y coordinates], not an array of (x,y) pairs.
        for (int i = 1; i < points[0].length; i++) {
            double x0 = points[0][i - 1], y0 = points[1][i - 1];
            double x1 = points[0][i], y1 = points[1][i];
            if (TuningMath.isFinite(x0) && TuningMath.isFinite(y0)
                    && TuningMath.isFinite(x1) && TuningMath.isFinite(y1)) {
                panelsField.moveCursor(x0, y0);
                panelsField.line(x1, y1);
            }
        }
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}

/** Pure numerical validation shared by the tuners; these methods never command hardware. */
final class TuningMath {
    // Numerical resolution, not a robot calibration or a physical motion tolerance.
    private static final double EPSILON = 1e-9;

    private TuningMath() {}

    static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    static boolean validPose(Pose pose) {
        return pose != null && isFinite(pose.getX()) && isFinite(pose.getY())
                && isFinite(pose.getHeading());
    }

    static boolean validVelocity(Vector velocity) {
        return velocity != null && isFinite(velocity.getXComponent())
                && isFinite(velocity.getYComponent()) && isFinite(velocity.getMagnitude());
    }

    /** Signed displacement along the initial robot's forward or left (+lateral) axis. */
    static double displacement(Pose start, Pose current, boolean lateral) {
        if (!validPose(start) || !validPose(current)) return Double.NaN;
        double dx = current.getX() - start.getX();
        double dy = current.getY() - start.getY();
        double heading = start.getHeading();
        double distance = lateral ? -dx * Math.sin(heading) + dy * Math.cos(heading)
                : dx * Math.cos(heading) + dy * Math.sin(heading);
        return isFinite(distance) ? distance : Double.NaN;
    }

    /** Returns NaN unless the observed motion agrees with the requested signed motion. */
    static double multiplier(double target, double measured, double currentMultiplier) {
        if (!isFinite(target) || !isFinite(measured) || !isFinite(currentMultiplier)
                || Math.abs(target) <= EPSILON || Math.abs(measured) <= EPSILON
                || currentMultiplier <= 0 || Math.signum(target) != Math.signum(measured)) {
            return Double.NaN;
        }
        double result = (target / measured) * currentMultiplier;
        return isFinite(result) && result > 0 ? result : Double.NaN;
    }

    /** Dashboard sample counts must be positive integers, never silently rounded. */
    static int sampleCount(double requested) {
        if (!isFinite(requested) || requested <= 0 || requested > Integer.MAX_VALUE
                || requested != Math.rint(requested)) {
            throw new IllegalArgumentException("Sample count must be a finite positive integer");
        }
        return (int) requested;
    }

    /** Mean of actual finite measurements; callers must also validate the physical sign. */
    static double mean(List<Double> samples, int minimumSamples) {
        if (minimumSamples <= 0 || samples == null || samples.size() < minimumSamples) {
            throw new IllegalArgumentException("Insufficient measurement samples");
        }
        double result = 0;
        for (Double sample : samples) {
            if (sample == null || !isFinite(sample)) {
                throw new IllegalArgumentException("Measurement sample is not finite");
            }
            result += sample / samples.size();
        }
        if (!isFinite(result)) throw new IllegalArgumentException("Measurement average is not finite");
        return result;
    }

    /** Validates a physical distance = linear*v + quadratic*v^2 fit; returns {linear, quadratic}. */
    static double[] brakingFit(List<double[]> samples) {
        if (samples == null || samples.size() < 2) {
            throw new IllegalArgumentException("Braking fit needs at least two distinct speeds");
        }
        double sum2 = 0, sum3 = 0, sum4 = 0, sumXY = 0, sumX2Y = 0;
        for (double[] sample : samples) {
            if (sample == null || sample.length < 2 || !isFinite(sample[0])
                    || !isFinite(sample[1]) || sample[0] <= 0 || sample[1] <= 0) {
                throw new IllegalArgumentException("Braking samples need finite positive speed and distance");
            }
            double x = sample[0], y = sample[1], x2 = x * x;
            sum2 += x2;
            sum3 += x2 * x;
            sum4 += x2 * x2;
            sumXY += x * y;
            sumX2Y += x2 * y;
        }
        double diagonalProduct = sum2 * sum4;
        double offDiagonalProduct = sum3 * sum3;
        double determinant = diagonalProduct - offDiagonalProduct;
        if (!isFinite(sumXY) || !isFinite(sumX2Y) || !isFinite(diagonalProduct)
                || !isFinite(offDiagonalProduct) || !isFinite(determinant)
                || determinant <= EPSILON * Math.max(diagonalProduct, offDiagonalProduct)
                || determinant < 1e-10) {
            throw new IllegalArgumentException("Braking samples produce a singular or unstable fit");
        }
        final double[] coefficients;
        try {
            coefficients = com.pedropathing.math.MathFunctions.quadraticFit(samples);
        } catch (RuntimeException failure) {
            throw new IllegalArgumentException("Braking fit could not be calculated", failure);
        }
        if (coefficients == null || coefficients.length != 2 || !isFinite(coefficients[0])
                || !isFinite(coefficients[1]) || coefficients[0] < 0 || coefficients[1] < 0
                || (coefficients[0] == 0 && coefficients[1] == 0)) {
            throw new IllegalArgumentException("Braking fit produced invalid physical coefficients");
        }
        return coefficients;
    }
}
