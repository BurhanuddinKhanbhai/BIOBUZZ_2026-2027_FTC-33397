
package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

/**
 * BIOBUZZ Red Autonomous route.
 *
 * Starting pose: (0, 0, 0)
 * +X = initially forward
 * +Y = initially left
 * Positive heading = counterclockwise
 *
 * The route is relative to the robot's starting position.
 * No absolute field-boundary checking is performed.
 */
public final class RedAutoPlan {

    public enum Step {
        SHOOT_1,
        TURN_LEFT_90,
        INTAKE_1,
        RETURN_1,
        TURN_RIGHT_90,
        DRIVE_TO_SHOOT_2,
        TURN_180_1,
        SHOOT_2,
        FLOWER_DOWN,
        TURN_180_2,
        BACK_HALF_TILE_1,
        STRAFE_LEFT,
        INTAKE_2,
        BACK_HALF_TILE_2,
        TURN_180_3,
        SHOOT_3
    }

    private final Pose start;
    private final Pose[] targets =
            new Pose[Step.values().length];

    public RedAutoPlan(double intake2Distance) {
        requirePositive(intake2Distance, "INTAKE_2 distance");

        start = new Pose(0, 0, 0);
        Pose planned = start;

        for (Step step : Step.values()) {
            if (isTurn(step)) {
                planned = new Pose(
                        planned.getX(),
                        planned.getY(),
                        normalize(
                                planned.getHeading()
                                        + turnRadians(step)
                        )
                );
            } else {
                switch (step) {
                    case INTAKE_1:
                        planned = translate(planned, 48.0, 0);
                        break;

                    case RETURN_1:
                        planned = translate(planned, -72.0, 0);
                        break;

                    case DRIVE_TO_SHOOT_2:
                        planned = translate(planned, 120.0, 0);
                        break;

                    case BACK_HALF_TILE_1:
                    case BACK_HALF_TILE_2:
                        planned = translate(planned, -12.0, 0);
                        break;

                    case STRAFE_LEFT:
                        planned = translate(planned, 0, 12.0);
                        break;

                    case INTAKE_2:
                        planned = translate(
                                planned, intake2Distance, 0
                        );
                        break;

                    default:
                        break;
                }
            }

            targets[step.ordinal()] = planned;
        }
    }

    public Pose getStart() {
        return start;
    }

    public Pose getTarget(Step step) {
        if (step == null) {
            throw new IllegalArgumentException(
                    "Autonomous step is required"
            );
        }

        return targets[step.ordinal()];
    }

    public boolean contains(Pose pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getHeading());
    }

    public static boolean isDrive(Step step) {
        return step == Step.INTAKE_1
                || step == Step.RETURN_1
                || step == Step.DRIVE_TO_SHOOT_2
                || step == Step.BACK_HALF_TILE_1
                || step == Step.STRAFE_LEFT
                || step == Step.INTAKE_2
                || step == Step.BACK_HALF_TILE_2;
    }

    public static boolean isTurn(Step step) {
        return step == Step.TURN_LEFT_90
                || step == Step.TURN_RIGHT_90
                || step == Step.TURN_180_1
                || step == Step.TURN_180_2
                || step == Step.TURN_180_3;
    }

    public static boolean isShoot(Step step) {
        return step == Step.SHOOT_1
                || step == Step.SHOOT_2
                || step == Step.SHOOT_3;
    }

    public static double turnRadians(Step step) {
        if (step == Step.TURN_LEFT_90) {
            return Math.PI / 2.0;
        }

        if (step == Step.TURN_RIGHT_90) {
            return -Math.PI / 2.0;
        }

        if (step == Step.TURN_180_1
                || step == Step.TURN_180_2
                || step == Step.TURN_180_3) {
            return Math.PI;
        }

        throw new IllegalArgumentException(
                "Not a turn step: " + step
        );
    }

    private static Pose translate(
            Pose from,
            double forward,
            double left
    ) {
        double cos = cardinal(Math.cos(from.getHeading()));
        double sin = cardinal(Math.sin(from.getHeading()));

        return new Pose(
                from.getX() + forward * cos - left * sin,
                from.getY() + forward * sin + left * cos,
                from.getHeading()
        );
    }

    private static double cardinal(double value) {
        return Math.abs(value) < 1e-12 ? 0.0 : value;
    }

    private static double normalize(double radians) {
        return (radians % (2.0 * Math.PI) + 2.0 * Math.PI)
                % (2.0 * Math.PI);
    }

    private static void requirePositive(
            double value,
            String name
    ) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    name + " must be finite and positive"
            );
        }
    }
}
