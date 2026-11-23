package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Consumer;

public class Constants {
    // Existing Pedro settings are retained, but are NOT certified robot calibration.
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Only change this in a reviewed hardware/calibration task. Tuning remains disabled.


    // Supply the localizer's validated configuration only after measuring wheel diameter,
    // counts/revolution, gearing, all three signs, signed offsets, multipliers and sample limits.
    // BIOBUZZ measured odometry pod offsets (inches)
    private static final double LEFT_Y = 6.25;
    private static final double RIGHT_Y = -6.25;
    private static final double PERPENDICULAR_X = -3.5;
    public static SafeThreeWheelLocalizer.Configuration odometryConfiguration =
            new SafeThreeWheelLocalizer.Configuration(
                    8192.0,       // Encoder ticks per revolution
                    35.0 / 25.4,  // Wheel diameter in inches
                    1.0,          // Gear ratio

                    1, 1, 1,      // Left, right, perpendicular encoder directions

                    LEFT_Y,
                    RIGHT_Y,
                    PERPENDICULAR_X,

                    1.0, 1.0, 1.0,  // Initial distance correction multipliers

                    100000.0,     // Provisional maximum encoder ticks/sec
                    0.5           // Maximum sampling interval in seconds
            );

    public static double mecanumForwardVelocity = 81.34056;
    public static double mecanumLateralVelocity = 65.43028;
    public static double mecanumMaxPower = 0.25;
    public static double mecanumMotorCachingThreshold = 0.01;

    // These names must be assigned to the stated ports in the Robot Controller configuration.
    private static final String FRONT_LEFT = "frontLeft";   // Expansion Hub (left), motor/encoder 0
    private static final String BACK_LEFT = "backLeft";     // Expansion Hub (left), motor 1
    private static final String FRONT_RIGHT = "frontRight"; // Control Hub (right), motor/encoder 0
    private static final String BACK_RIGHT = "backRight";   // Control Hub (right), motor 1
    private static final String PERPENDICULAR = "transfer"; // Expansion Hub (left), motor/encoder 3

    /**
     * Read-only preflight for OpMode telemetry. Returns null only when configuration and
     * the review guard pass; never accesses hardware or constructs a follower.
     * HardwareMap grouping, ports, stopped motors and modes are still checked by the factory.
     */
    public static String getConfigurationIssue() {
        StringBuilder issues = new StringBuilder();
        if (odometryConfiguration == null) {
            appendIssue(issues, "odometry wheel/encoder data, three signs, signed offsets, correction factors and sample limits");
        }
        if (!Double.isFinite(mecanumForwardVelocity) || mecanumForwardVelocity <= 0.0) {
            appendIssue(issues, "measured mecanum forward velocity");
        }
        if (!Double.isFinite(mecanumLateralVelocity) || mecanumLateralVelocity <= 0.0) {
            appendIssue(issues, "measured mecanum lateral velocity");
        }
        if (!Double.isFinite(mecanumMaxPower) || mecanumMaxPower <= 0.0 || mecanumMaxPower > 1.0) {
            appendIssue(issues, "verified mecanum maximum power in (0, 1]");
        }
        if (!Double.isFinite(mecanumMotorCachingThreshold) || mecanumMotorCachingThreshold < 0.0
                || (Double.isFinite(mecanumMaxPower) && mecanumMotorCachingThreshold >= mecanumMaxPower)) {
            appendIssue(issues, "motor caching threshold (nonnegative and below maximum power)");
        }
        if (followerConstants == null || pathConstraints == null) {
            appendIssue(issues, "follower/path settings");
        }
        if (issues.length() == 0) {
            try {
                validateCalibration();
            } catch (IllegalStateException invalid) {
                appendIssue(issues, invalid.getMessage());
            }
        }

        return issues.length() == 0 ? null : issues.toString();
    }

    private static void appendIssue(StringBuilder issues, String issue) {
        if (issues.length() != 0) issues.append("; ");
        issues.append(issue);
    }

    /**
     * Guarded integration only: never enable this factory by supplying guessed calibration.
     * Before a future handover, initialize/stop the normal drivetrain and transfer owners.
     * Their modes/directions must remain stable; never drive through two owners concurrently.
     * Numeric validation and the review guard precede ALL HardwareMap/device access.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, null);
    }

    /**
     * Optional field-clearance validation runs on each fresh pose BEFORE Pedro can write
     * driving output. Validation failure throws; the caller must latch failure and stop
     * every motor independently. It must not retry a failed active motion automatically.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Consumer<Pose> poseValidator) {
        validateCalibration();
        require(hardwareMap != null, "HardwareMap is required after configuration verification");

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT);
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);
        DcMotorEx perpendicular = hardwareMap.get(DcMotorEx.class, PERPENDICULAR);
        validateHardware(frontLeft, backLeft, frontRight, backRight, perpendicular);

        // Pedro 2.1.2 Mecanum does NOT set run modes or reset encoders. It DOES clone
        // motor metadata (max RPM fraction), stop motors and select FLOAT. updateConstants()
        // applies directions, so call it before capturing any shared encoder baselines.
        Mecanum drivetrain = new Mecanum(hardwareMap, createMecanumConstants());
        drivetrain.updateConstants();
        SafeThreeWheelLocalizer localizer = new SafeThreeWheelLocalizer(
                frontLeft, frontRight, perpendicular, odometryConfiguration);
        // Preserve FollowerBuilder.pathConstraints()'s default-constraint behavior.
        PathConstraints.setDefaultConstraints(pathConstraints);
        Follower follower = new Follower(followerConstants, localizer, drivetrain, pathConstraints) {
            @Override
            public void updatePose() {
                super.updatePose();
                // Pedro 2.1.2 does not check Localizer.isNAN() before runDrive(). A rejected
                // localizer sample retains its last finite pose, so a post-update check is late.
                if (!localizer.isSampleValid() || localizer.isNAN()) {
                    throw new IllegalStateException("Invalid odometry feedback: " + localizer.getStatus());
                }
                if (poseValidator != null) poseValidator.accept(getPose());
            }
        };
        follower.setMaxPower(mecanumMaxPower);
        // Follower construction calls breakFollowing(), which selects FLOAT. Restore the
        // existing DriveTrain's stopped BRAKE behavior without entering follower TeleOp mode.
        // Later Pedro breakFollowing() calls still select FLOAT; braking tuning must account for it.
        drivetrain.startTeleopDrive(true);
        return follower;
    }

    private static MecanumConstants createMecanumConstants() {
        MecanumConstants settings = new MecanumConstants()
                .leftFrontMotorName(FRONT_LEFT).leftRearMotorName(BACK_LEFT)
                .rightFrontMotorName(FRONT_RIGHT).rightRearMotorName(BACK_RIGHT)
                // Match the current DriveTrain constructor used by competition TeleOp.
                // Encoder signs must be calibrated with these motor directions in place.
                .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .xVelocity(mecanumForwardVelocity).yVelocity(mecanumLateralVelocity)
                .maxPower(mecanumMaxPower).motorCachingThreshold(mecanumMotorCachingThreshold)
                .useBrakeModeInTeleOp(true).useVoltageCompensation(false);
        // The 2.1.2 xVelocity/yVelocity setters do not rebuild this vector. Derive it
        // explicitly from calibrated speeds rather than retaining Pedro's default ratio.
        settings.setFrontLeftVector(new Vector(1.0,
                Math.atan2(-mecanumLateralVelocity, mecanumForwardVelocity)));
        return settings;
    }

    private static void validateCalibration() {
        require(odometryConfiguration != null,
                "SafeThreeWheelLocalizer calibration is missing (wheel/encoder data, signs, offsets, multipliers, sample limits)");
        // Configuration is immutable and rejects invalid values in its own constructor.
        requirePositive("mecanum forward velocity", mecanumForwardVelocity);
        requirePositive("mecanum lateral velocity", mecanumLateralVelocity);
        requirePositive("mecanum maximum power", mecanumMaxPower);
        require(mecanumMaxPower <= 1.0, "mecanum maximum power must be at most 1");
        requireFinite("motor caching threshold", mecanumMotorCachingThreshold);
        require(mecanumMotorCachingThreshold >= 0.0 && mecanumMotorCachingThreshold < mecanumMaxPower,
                "motor caching threshold must be nonnegative and less than maximum power");
        require(followerConstants != null && pathConstraints != null, "follower/path settings are missing");

        FollowerConstants f = followerConstants;
        requirePositive("robot mass", f.mass);
        requireFinite("zero-power accelerations", f.forwardZeroPowerAcceleration, f.lateralZeroPowerAcceleration);
        require(f.forwardZeroPowerAcceleration < 0.0 && f.lateralZeroPowerAcceleration < 0.0,
                "zero-power accelerations must be negative");
        validatePIDF(f.coefficientsTranslationalPIDF);
        validatePIDF(f.integralTranslational);
        validatePIDF(f.coefficientsHeadingPIDF);
        validatePIDF(f.coefficientsSecondaryTranslationalPIDF);
        validatePIDF(f.integralSecondaryTranslational);
        validatePIDF(f.coefficientsSecondaryHeadingPIDF);
        validateDrivePIDF(f.coefficientsDrivePIDF);
        validateDrivePIDF(f.coefficientsSecondaryDrivePIDF);
        require(f.predictiveBrakingCoefficients != null, "predictive braking coefficients are missing");
        requireFinite("predictive braking coefficients", f.predictiveBrakingCoefficients.kLinearBraking,
                f.predictiveBrakingCoefficients.kQuadraticFriction, f.predictiveBrakingCoefficients.P,
                f.predictiveBrakingCoefficients.maximumBrakingPower);
        requireFinite("follower settings", f.headingPIDFSwitch, f.drivePIDFSwitch, f.translationalPIDFSwitch,
                f.holdPointTranslationalScaling, f.holdPointHeadingScaling, f.turnHeadingErrorThreshold,
                f.centripetalScaling, f.driveKalmanFilterModelCovariance, f.driveKalmanFilterDataCovariance,
                f.stuckVelocity, f.stuckTValueLow, f.stuckTValueHigh, f.stuckTimeout);
        require(f.BEZIER_CURVE_SEARCH_LIMIT > 0, "follower search limit must be positive");
        require(f.driveKalmanFilterModelCovariance >= 0.0 && f.driveKalmanFilterDataCovariance > 0.0,
                "Kalman covariance must have nonnegative model and positive measurement values");

        requireFinite("path settings", pathConstraints.getTValueConstraint(), pathConstraints.getVelocityConstraint(),
                pathConstraints.getTranslationalConstraint(), pathConstraints.getHeadingConstraint(),
                pathConstraints.getTimeoutConstraint(), pathConstraints.getBrakingStrength(), pathConstraints.getBrakingStart());
        require(pathConstraints.getTValueConstraint() > 0.0 && pathConstraints.getTValueConstraint() <= 1.0
                        && pathConstraints.getVelocityConstraint() >= 0.0 && pathConstraints.getTranslationalConstraint() >= 0.0
                        && pathConstraints.getHeadingConstraint() >= 0.0 && pathConstraints.getTimeoutConstraint() > 0.0
                        && pathConstraints.getBrakingStrength() > 0.0 && pathConstraints.getBrakingStart() > 0.0
                        && pathConstraints.getBrakingStart() <= 1.0 && pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT() > 0,
                "path constraints contain invalid limits");
    }

    /** Read-only preflight. Hub identity is relative; physical Control/Expansion labels require RC setup verification. */
    private static void validateHardware(DcMotorEx frontLeft, DcMotorEx backLeft,
                                         DcMotorEx frontRight, DcMotorEx backRight, DcMotorEx perpendicular) {
        require(frontLeft.getController() != null && frontRight.getController() != null
                        && frontLeft.getController() != frontRight.getController()
                        && backLeft.getController() == frontLeft.getController()
                        && perpendicular.getController() == frontLeft.getController()
                        && backRight.getController() == frontRight.getController(),
                "motor names do not match the planned two-hub grouping");
        require(frontLeft.getPortNumber() == 0 && backLeft.getPortNumber() == 1
                        && frontRight.getPortNumber() == 0 && backRight.getPortNumber() == 1
                        && perpendicular.getPortNumber() == 3,
                "expected left Expansion Hub frontLeft=0/backLeft=1/transfer=3; right Control Hub frontRight=0/backRight=1");
        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight, perpendicular}) {
            require(motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                    "drivetrain and transfer must already use RUN_WITHOUT_ENCODER; the factory will not change shared-channel modes");
            require(motor.getPower() == 0.0, "drivetrain/transfer owners must stop motors before follower handover");
        }
        require(frontLeft.getDirection() == DcMotorSimple.Direction.FORWARD
                        && backLeft.getDirection() == DcMotorSimple.Direction.FORWARD
                        && frontRight.getDirection() == DcMotorSimple.Direction.REVERSE
                        && backRight.getDirection() == DcMotorSimple.Direction.REVERSE,
                "motor directions must match DriveTrain before follower handover");
        // Do not configure transfer: it only supplies perpendicular counts here. Its direction
        // must be established by its owner before encoder signs are calibrated/baselined.
    }

    private static void validatePIDF(PIDFCoefficients coefficients) {
        require(coefficients != null, "PIDF coefficients are missing");
        requireFinite("PIDF coefficients", coefficients.P, coefficients.I, coefficients.D, coefficients.F);
    }

    private static void validateDrivePIDF(FilteredPIDFCoefficients coefficients) {
        require(coefficients != null, "drive PIDF coefficients are missing");
        requireFinite("drive PIDF coefficients", coefficients.P, coefficients.I, coefficients.D, coefficients.F, coefficients.T);
        require(coefficients.T >= 0.0 && coefficients.T <= 1.0, "drive filter T must be in [0, 1]");
    }

    private static void requirePositive(String name, double value) {
        require(Double.isFinite(value) && value > 0.0, name + " must be explicitly configured, finite and positive");
    }

    private static void requireFinite(String name, double... values) {
        for (double value : values) {
            require(Double.isFinite(value), name + " must be finite");
        }
    }

    private static void require(boolean condition, String message) {
        if (!condition) {
            throw new IllegalStateException("Pedro Pathing initialization disabled: " + message);
        }
    }
}
