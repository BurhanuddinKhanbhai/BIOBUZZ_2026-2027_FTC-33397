package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // Pedro tuning and odometry measurements are retained from the current robot configuration.
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // BIOBUZZ measured odometry pod offsets (inches)
    private static final double LEFT_Y = 6.25;
    private static final double RIGHT_Y = -6.25;
    private static final double PERPENDICULAR_X = -3.5;
    public static SafeThreeWheelLocalizer.Configuration odometryConfiguration =
            new SafeThreeWheelLocalizer.Configuration(
                    8192.0,       // Encoder ticks per revolution
                    35.0 / 25.4,  // Wheel diameter in inches
                    1.0,          // Gear ratio

                    -1, 1, -1,      // Left, right, perpendicular encoder directions

                    LEFT_Y,
                    RIGHT_Y,
                    PERPENDICULAR_X,

                    1.1032, 1.1268, 1.1062  // Distance correction multipliers
            );

    public static double mecanumForwardVelocity = 81.34056;
    public static double mecanumLateralVelocity = 65.43028;
    public static double mecanumMaxPower = 1;
    public static double mecanumMotorCachingThreshold = 0.01;

    // These names must be assigned to the stated ports in the Robot Controller configuration.
    private static final String FRONT_LEFT = "frontLeft";   // Expansion Hub (left), motor/encoder 0
    private static final String BACK_LEFT = "backLeft";     // Expansion Hub (left), motor 1
    private static final String FRONT_RIGHT = "frontRight"; // Control Hub (right), motor/encoder 0
    private static final String BACK_RIGHT = "backRight";   // Control Hub (right), motor 1
    private static final String PERPENDICULAR = "intake";    // Control Hub (right), motor/encoder 2

    /** Initialize motor owners before creating the follower so shared encoder directions remain stable. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT);
        DcMotorEx perpendicular = hardwareMap.get(DcMotorEx.class, PERPENDICULAR);

        // Pedro 2.1.2 Mecanum does NOT set run modes or reset encoders. It DOES clone
        // motor metadata (max RPM fraction), stop motors and select FLOAT. updateConstants()
        // applies directions, so call it before capturing any shared encoder baselines.
        Mecanum drivetrain = new Mecanum(hardwareMap, createMecanumConstants());
        drivetrain.updateConstants();
        SafeThreeWheelLocalizer localizer = new SafeThreeWheelLocalizer(
                frontLeft, frontRight, perpendicular, odometryConfiguration);
        // Preserve FollowerBuilder.pathConstraints()'s default-constraint behavior.
        PathConstraints.setDefaultConstraints(pathConstraints);
        Follower follower = new Follower(followerConstants, localizer, drivetrain, pathConstraints);
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

}
