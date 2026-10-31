package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Objects;
import java.util.function.LongSupplier;

/**
 * Three passive pods, read without taking ownership of their shared motor channels.
 * Distances are inches; robot +X is forward, +Y is left, and positive heading is
 * counterclockwise. Parallel pods measure +X and the perpendicular pod measures +Y.
 * All hardware access is through getCurrentPosition(); resets only capture baselines.
 *
 * Encoder signs apply to SDK-reported counts, which already reflect the motor's
 * configured direction/type. Configure those motors first and keep their configuration
 * stable while localizing. After any external counter/configuration change, explicitly
 * rebaseline with setPose(). A hardware reset producing a plausible delta cannot be
 * distinguished from real motion. Bounds also cannot detect arbitrary multiple wraps.
 *
 * Call from one control-loop thread. Rejected samples leave the last accepted pose
 * intact, clear velocity, and flag invalid feedback through isSampleValid()/isNAN().
 * This class does not stop motors; its caller must handle invalid localization feedback.
 */
public final class SafeThreeWheelLocalizer implements Localizer {
    /** No physical dimensions, directions, calibration factors, or motion limits default. */
    public static final class Configuration {
        private final int leftSign;
        private final int rightSign;
        private final int perpendicularSign;
        private final double leftY;
        private final double rightY;
        private final double perpendicularX;
        private final double separation;
        private final double forwardScale;
        private final double lateralScale;
        private final double turningScale;
        private final double maxTicksPerSecond;
        private final double maxSampleIntervalSeconds;

        /**
         * All pods use the supplied wheel/encoder specification. ticksPerRevolution is
         * the count reported by the SDK per encoder-shaft revolution; gearRatio is
         * encoder revolutions per odometry-wheel revolution. Signs must be +1 or -1.
         * Offsets are signed distances from the robot's chosen tracking origin.
         * Corrections are positive, dimensionless factors applied to the nominal
         * inches-per-tick scale (including a separate angular calibration factor).
         *
         * maxTicksPerSecond is an explicit plausible bound for EACH pod, including
         * rotation; a one-count quantization allowance is added when checking samples.
         * maxSampleIntervalSeconds limits gaps that may hide motion/counter wraps.
         */
        public Configuration(double ticksPerRevolution, double wheelDiameterInches,
                             double gearRatio, int leftSign, int rightSign,
                             int perpendicularSign, double leftYInches, double rightYInches,
                             double perpendicularXInches, double forwardCorrection,
                             double lateralCorrection, double turningCorrection,
                             double maxTicksPerSecond, double maxSampleIntervalSeconds) {
            requirePositive(ticksPerRevolution, "ticksPerRevolution");
            requirePositive(wheelDiameterInches, "wheelDiameterInches");
            requirePositive(gearRatio, "gearRatio");
            requireSign(leftSign, "leftSign");
            requireSign(rightSign, "rightSign");
            requireSign(perpendicularSign, "perpendicularSign");
            requireFinite(leftYInches, "leftYInches");
            requireFinite(rightYInches, "rightYInches");
            requireFinite(perpendicularXInches, "perpendicularXInches");
            requirePositive(forwardCorrection, "forwardCorrection");
            requirePositive(lateralCorrection, "lateralCorrection");
            requirePositive(turningCorrection, "turningCorrection");
            requirePositive(maxTicksPerSecond, "maxTicksPerSecond");
            requirePositive(maxSampleIntervalSeconds, "maxSampleIntervalSeconds");

            separation = leftYInches - rightYInches;
            if (!Double.isFinite(separation) || separation == 0.0) {
                throw new IllegalArgumentException("Parallel pod positions must have finite, nonzero separation");
            }
            double nominalScale = (wheelDiameterInches * Math.PI / ticksPerRevolution) / gearRatio;
            forwardScale = nominalScale * forwardCorrection;
            lateralScale = nominalScale * lateralCorrection;
            turningScale = nominalScale * turningCorrection;
            requirePositive(forwardScale, "forward inches per tick");
            requirePositive(lateralScale, "lateral inches per tick");
            requirePositive(turningScale, "turning inches per tick");
            // Unambiguous modular subtraction requires strictly less than half a wrap.
            if (!(maxTicksPerSecond * maxSampleIntervalSeconds + 1.0 < Integer.MAX_VALUE)) {
                throw new IllegalArgumentException("Tick-rate/interval bounds must allow less than half a counter wrap");
            }
            this.leftSign = leftSign;
            this.rightSign = rightSign;
            this.perpendicularSign = perpendicularSign;
            leftY = leftYInches;
            rightY = rightYInches;
            perpendicularX = perpendicularXInches;
            this.maxTicksPerSecond = maxTicksPerSecond;
            this.maxSampleIntervalSeconds = maxSampleIntervalSeconds;
        }
    }

    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx perpendicularEncoder;
    private final Configuration configuration;
    private final LongSupplier nanoClock;
    private int previousLeft;
    private int previousRight;
    private int previousPerpendicular;
    private long previousTime;
    private boolean hasBaseline;
    private boolean sampleValid;
    private String status;
    private Pose startPose = new Pose();
    private Pose pose = new Pose();
    private Pose velocity = new Pose();
    private double totalHeading;

    public SafeThreeWheelLocalizer(DcMotorEx leftEncoder, DcMotorEx rightEncoder,
                                  DcMotorEx perpendicularEncoder, Configuration configuration) {
        this(leftEncoder, rightEncoder, perpendicularEncoder, configuration, System::nanoTime);
    }

    /** The clock overload permits deterministic tests; supply a monotonic nanosecond clock. */
    public SafeThreeWheelLocalizer(DcMotorEx leftEncoder, DcMotorEx rightEncoder,
                                  DcMotorEx perpendicularEncoder, Configuration configuration,
                                  LongSupplier nanoClock) {
        this.leftEncoder = Objects.requireNonNull(leftEncoder, "leftEncoder");
        this.rightEncoder = Objects.requireNonNull(rightEncoder, "rightEncoder");
        this.perpendicularEncoder = Objects.requireNonNull(perpendicularEncoder, "perpendicularEncoder");
        if (leftEncoder == rightEncoder || leftEncoder == perpendicularEncoder
                || rightEncoder == perpendicularEncoder) {
            throw new IllegalArgumentException("Three distinct encoder sources are required");
        }
        this.configuration = Objects.requireNonNull(configuration, "configuration");
        this.nanoClock = Objects.requireNonNull(nanoClock, "nanoClock");
        captureBaseline();
    }

    @Override
    public void update() {
        final int left;
        final int right;
        final int perpendicular;
        final long now;
        try {
            left = leftEncoder.getCurrentPosition();
            right = rightEncoder.getCurrentPosition();
            perpendicular = perpendicularEncoder.getCurrentPosition();
            now = nanoClock.getAsLong();
        } catch (RuntimeException failure) {
            // A partial read cannot form a coherent baseline. Discard the next full sample.
            hasBaseline = false;
            reject("Encoder/clock read failed: " + failure);
            return;
        }
        if (!hasBaseline) {
            storeBaseline(left, right, perpendicular, now);
            reject("Baseline recovered; awaiting a complete motion interval");
            return;
        }

        // Subtract as int for modulo-2^32 wrap handling, then widen before applying signs.
        long leftTicks = (long) (left - previousLeft) * configuration.leftSign;
        long rightTicks = (long) (right - previousRight) * configuration.rightSign;
        long perpendicularTicks = (long) (perpendicular - previousPerpendicular)
                * configuration.perpendicularSign;
        double dt = (now - previousTime) * 1.0e-9;
        // Advance counts AND time even on rejection: rejected movement is never replayed.
        storeBaseline(left, right, perpendicular, now);
        if (!Double.isFinite(dt) || dt <= 0.0 || dt > configuration.maxSampleIntervalSeconds) {
            reject("Invalid or excessive sample interval");
            return;
        }
        double tickLimit = configuration.maxTicksPerSecond * dt + 1.0;
        if (Math.abs(leftTicks) > tickLimit || Math.abs(rightTicks) > tickLimit
                || Math.abs(perpendicularTicks) > tickLimit) {
            reject("Implausible encoder delta");
            return;
        }

        double headingDelta = (rightTicks - leftTicks) * configuration.turningScale
                / configuration.separation;
        double forward = (rightTicks * configuration.leftY - leftTicks * configuration.rightY)
                * configuration.forwardScale / configuration.separation;
        double lateral = perpendicularTicks * configuration.lateralScale
                - configuration.perpendicularX * headingDelta;
        if (!Double.isFinite(forward) || !Double.isFinite(lateral) || !Double.isFinite(headingDelta)) {
            reject("Nonfinite displacement");
            return;
        }

        // SE(2) exponential: integrate the body twist along an arc, including near-zero turns.
        double sinc;
        double cosc;
        if (Math.abs(headingDelta) < 1.0e-4) {
            double squared = headingDelta * headingDelta;
            sinc = 1.0 - squared / 6.0 + squared * squared / 120.0;
            cosc = headingDelta * (0.5 - squared / 24.0 + squared * squared / 720.0);
        } else {
            sinc = Math.sin(headingDelta) / headingDelta;
            cosc = (1.0 - Math.cos(headingDelta)) / headingDelta;
        }
        double bodyX = sinc * forward - cosc * lateral;
        double bodyY = cosc * forward + sinc * lateral;
        double cosine = Math.cos(pose.getHeading());
        double sine = Math.sin(pose.getHeading());
        double fieldX = cosine * bodyX - sine * bodyY;
        double fieldY = sine * bodyX + cosine * bodyY;
        Pose nextPose = new Pose(pose.getX() + fieldX, pose.getY() + fieldY,
                pose.getHeading() + headingDelta);
        Pose nextVelocity = new Pose(fieldX / dt, fieldY / dt, headingDelta / dt);
        double nextTotalHeading = totalHeading + headingDelta;
        if (!finitePose(nextPose) || !finitePose(nextVelocity) || !Double.isFinite(nextTotalHeading)
                || !Double.isFinite(Math.hypot(nextVelocity.getX(), nextVelocity.getY()))) {
            reject("Nonfinite pose or velocity");
            return;
        }
        pose = nextPose;
        velocity = nextVelocity;
        totalHeading = nextTotalHeading;
        sampleValid = true;
        status = "OK";
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    /** Field-frame average velocity over the last accepted interval; heading is radians/second. */
    @Override
    public Pose getVelocity() {
        return velocity;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(Math.hypot(velocity.getX(), velocity.getY()),
                Math.atan2(velocity.getY(), velocity.getX()));
    }

    /** Pedro semantics: shift the start while preserving accumulated field-frame displacement. */
    @Override
    public void setStartPose(Pose newStart) {
        Pose checkedStart = checkedPose(newStart);
        Pose relocated = checkedPose(checkedStart.plus(pose.minus(startPose)));
        captureBaseline();
        startPose = checkedStart;
        pose = relocated;
    }

    /** Rebase current pose/counts; preserve the starting pose and accumulated measured turn. */
    @Override
    public void setPose(Pose newPose) {
        Pose checked = checkedPose(newPose);
        captureBaseline();
        pose = checked;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return configuration.forwardScale;
    }

    @Override
    public double getLateralMultiplier() {
        return configuration.lateralScale;
    }

    /** Like Pedro's three-wheel implementation: inches/tick, divided by pod separation for radians. */
    @Override
    public double getTurningMultiplier() {
        return configuration.turningScale;
    }

    /** No IMU is owned by this localizer. */
    @Override
    public void resetIMU() {
        // Intentionally no hardware action.
    }

    /** No IMU measurement is available. Encoder heading remains available through getPose(). */
    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    /** Also flags rejected feedback, even though the last accepted pose remains finite. */
    @Override
    public boolean isNAN() {
        return !sampleValid || !finitePose(pose) || !finitePose(velocity);
    }

    public boolean isSampleValid() {
        return sampleValid;
    }

    public String getStatus() {
        return status;
    }

    private void captureBaseline() {
        try {
            int left = leftEncoder.getCurrentPosition();
            int right = rightEncoder.getCurrentPosition();
            int perpendicular = perpendicularEncoder.getCurrentPosition();
            long now = nanoClock.getAsLong();
            storeBaseline(left, right, perpendicular, now);
            velocity = new Pose();
            sampleValid = true;
            status = "Baselined; no motion interval yet";
        } catch (RuntimeException failure) {
            hasBaseline = false;
            reject("Cannot establish software baseline: " + failure);
            throw failure;
        }
    }

    private void storeBaseline(int left, int right, int perpendicular, long now) {
        previousLeft = left;
        previousRight = right;
        previousPerpendicular = perpendicular;
        previousTime = now;
        hasBaseline = true;
    }

    private void reject(String reason) {
        velocity = new Pose();
        sampleValid = false;
        status = reason;
    }

    private static Pose checkedPose(Pose value) {
        Objects.requireNonNull(value, "pose");
        if (!finitePose(value)) {
            throw new IllegalArgumentException("Pose must be finite");
        }
        Pose converted = value.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        if (!finitePose(converted)) {
            throw new IllegalArgumentException("Converted Pedro pose must be finite");
        }
        return converted;
    }

    private static boolean finitePose(Pose value) {
        return Double.isFinite(value.getX()) && Double.isFinite(value.getY())
                && Double.isFinite(value.getHeading());
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
    }

    private static void requireSign(int value, String name) {
        if (value != 1 && value != -1) {
            throw new IllegalArgumentException(name + " must be +1 or -1");
        }
    }
}
