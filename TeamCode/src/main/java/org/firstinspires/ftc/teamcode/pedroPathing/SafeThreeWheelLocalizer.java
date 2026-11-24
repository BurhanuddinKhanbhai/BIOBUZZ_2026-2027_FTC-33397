package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.LongSupplier;

/**
 * Three passive pods read through their shared motor encoder channels.
 * Inches, robot +X forward, +Y left, and counterclockwise heading.
 * All hardware access is getCurrentPosition(); pose changes capture software baselines only.
 * Encoder signs apply to SDK counts, including the motor owner's configured direction.
 */
public final class SafeThreeWheelLocalizer implements Localizer {
    /** Measured wheel, encoder, signed-offset and distance-correction parameters. */
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

        /**
         * ticksPerRevolution is SDK counts per encoder revolution; gearRatio is encoder
         * revolutions per pod-wheel revolution. Offsets are relative to the tracking origin.
         */
        public Configuration(double ticksPerRevolution, double wheelDiameterInches,
                             double gearRatio, int leftSign, int rightSign,
                             int perpendicularSign, double leftYInches, double rightYInches,
                             double perpendicularXInches, double forwardCorrection,
                             double lateralCorrection, double turningCorrection) {
            separation = leftYInches - rightYInches;
            double nominalScale = (wheelDiameterInches * Math.PI / ticksPerRevolution) / gearRatio;
            forwardScale = nominalScale * forwardCorrection;
            lateralScale = nominalScale * lateralCorrection;
            turningScale = nominalScale * turningCorrection;
            this.leftSign = leftSign;
            this.rightSign = rightSign;
            this.perpendicularSign = perpendicularSign;
            leftY = leftYInches;
            rightY = rightYInches;
            perpendicularX = perpendicularXInches;
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
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.perpendicularEncoder = perpendicularEncoder;
        this.configuration = configuration;
        this.nanoClock = nanoClock;
        captureBaseline();
    }

    @Override
    public void update() {
        int left = leftEncoder.getCurrentPosition();
        int right = rightEncoder.getCurrentPosition();
        int perpendicular = perpendicularEncoder.getCurrentPosition();
        long now = nanoClock.getAsLong();

        // Subtract as int for modulo-2^32 wrap handling, then widen before applying signs.
        long leftTicks = (long) (left - previousLeft) * configuration.leftSign;
        long rightTicks = (long) (right - previousRight) * configuration.rightSign;
        long perpendicularTicks = (long) (perpendicular - previousPerpendicular)
                * configuration.perpendicularSign;
        double dt = (now - previousTime) * 1.0e-9;
        storeBaseline(left, right, perpendicular, now);

        double headingDelta = (rightTicks - leftTicks) * configuration.turningScale
                / configuration.separation;
        double forward = (rightTicks * configuration.leftY - leftTicks * configuration.rightY)
                * configuration.forwardScale / configuration.separation;
        double lateral = perpendicularTicks * configuration.lateralScale
                - configuration.perpendicularX * headingDelta;
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
        pose = new Pose(pose.getX() + fieldX, pose.getY() + fieldY,
                pose.getHeading() + headingDelta);
        // No velocity can be calculated between identical clock ticks; pose integration is retained.
        velocity = dt > 0.0 ? new Pose(fieldX / dt, fieldY / dt, headingDelta / dt) : new Pose();
        totalHeading += headingDelta;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    /** Field-frame average velocity over the last interval; heading is radians/second. */
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
        Pose checkedStart = newStart.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        Pose relocated = checkedStart.plus(pose.minus(startPose));
        captureBaseline();
        startPose = checkedStart;
        pose = relocated;
    }

    /** Rebase current pose/counts; preserve the starting pose and accumulated measured turn. */
    @Override
    public void setPose(Pose newPose) {
        Pose checked = newPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
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

    /** Required Localizer metadata; no custom gate or sample-rejection state. */
    @Override
    public boolean isNAN() {
        return !finitePose(pose) || !finitePose(velocity);
    }

    private void captureBaseline() {
        storeBaseline(leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition(),
                perpendicularEncoder.getCurrentPosition(), nanoClock.getAsLong());
        velocity = new Pose();
    }

    private void storeBaseline(int left, int right, int perpendicular, long now) {
        previousLeft = left;
        previousRight = right;
        previousPerpendicular = perpendicular;
        previousTime = now;
    }

    private static boolean finitePose(Pose value) {
        return Double.isFinite(value.getX()) && Double.isFinite(value.getY())
                && Double.isFinite(value.getHeading());
    }

}
