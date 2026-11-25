package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

/** Robot-relative Red autonomous route. Positive heading and strafe are left. */
public final class RedAutoPlan {
    // MEASURE LATER: edit these four values, in inches.
    // Forward: positive = robot forward, negative = robot backward.
    // Left: positive = robot left, negative = robot right.
    public static final double FLOWER_INTAKE_FORWARD_INCHES = 0.0;
    public static final double FLOWER_INTAKE_LEFT_INCHES = 0.0;
    public static final double PARK_FORWARD_INCHES = 0.0;
    public static final double PARK_LEFT_INCHES = 0.0;

    public enum Step {
        CLEAR_WALL_FORWARD,  // 2 inches, before opening shot
        SHOOT_1,
        TURN_LEFT_90,
        CLEAR_WALL_STRAFE,   // 1 inch robot-left (current measured value)
        INTAKE_1,           // 45 inches forward
        RETURN_1,           // 23 inches backward
        TURN_RIGHT_90,
        DRIVE_TO_SHOOT_2,   // 110 inches forward
        TURN_RIGHT_70_1,
        SHOOT_2,
        FLOWER_DOWN,
        TURN_LEFT_70,
        INTAKE_FLOWER,      // placeholder: 0 forward, 0 left
        TURN_RIGHT_70_2,
        SHOOT_3,
        TURN_RIGHT_200,
        PARK_FORWARD        // placeholder: 0 forward, 0 left
    }

    private final Pose start = new Pose(0, 0, 0);
    private final Pose[] targets = new Pose[Step.values().length];

    public RedAutoPlan() {
        this(FLOWER_INTAKE_FORWARD_INCHES, FLOWER_INTAKE_LEFT_INCHES,
                PARK_FORWARD_INCHES, PARK_LEFT_INCHES);
    }

    /** Custom values can be supplied for geometry tests without altering robot defaults. */
    public RedAutoPlan(double flowerForward, double flowerLeft,
                       double parkForward, double parkLeft) {
        Pose planned = start;
        for (Step step : Step.values()) {
            if (isTurn(step)) {
                planned = new Pose(planned.getX(), planned.getY(),
                        normalize(planned.getHeading() + turnRadians(step)));
            } else {
                switch (step) {
                    case CLEAR_WALL_FORWARD:
                        planned = translate(planned, 2.0, 0.0);
                        break;
                    case CLEAR_WALL_STRAFE:
                        planned = translate(planned, 0.0, 1.0);
                        break;
                    case INTAKE_1:
                        planned = translate(planned, 45.0, 0.0);
                        break;
                    case RETURN_1:
                        planned = translate(planned, -23.0, 0.0);
                        break;
                    case DRIVE_TO_SHOOT_2:
                        planned = translate(planned, 110.0, 0.0);
                        break;
                    case INTAKE_FLOWER:
                        planned = translate(planned, flowerForward, flowerLeft);
                        break;
                    case PARK_FORWARD:
                        planned = translate(planned, parkForward, parkLeft);
                        break;
                    default:
                        break; // Shooting and flower deployment do not change pose.
                }
            }
            targets[step.ordinal()] = planned;
        }
    }

    public Pose getStart() { return start; }
    public Pose getTarget(Step step) { return targets[step.ordinal()]; }

    public boolean isZeroDistanceDrive(Step step) {
        if (!isDrive(step)) return false;
        Pose previous = step.ordinal() == 0 ? start : targets[step.ordinal() - 1];
        Pose current = getTarget(step);
        return Math.hypot(current.getX() - previous.getX(),
                current.getY() - previous.getY()) < 1e-8;
    }

    public static boolean isDrive(Step step) {
        return step == Step.CLEAR_WALL_FORWARD
                || step == Step.CLEAR_WALL_STRAFE
                || step == Step.INTAKE_1
                || step == Step.RETURN_1
                || step == Step.DRIVE_TO_SHOOT_2
                || step == Step.INTAKE_FLOWER
                || step == Step.PARK_FORWARD;
    }

    public static boolean isTurn(Step step) {
        return step == Step.TURN_LEFT_90
                || step == Step.TURN_RIGHT_90
                || step == Step.TURN_RIGHT_70_1
                || step == Step.TURN_LEFT_70
                || step == Step.TURN_RIGHT_70_2
                || step == Step.TURN_RIGHT_200;
    }

    public static boolean isShoot(Step step) {
        return step == Step.SHOOT_1 || step == Step.SHOOT_2 || step == Step.SHOOT_3;
    }

    public static double turnRadians(Step step) {
        switch (step) {
            case TURN_LEFT_90: return Math.PI / 2.0;
            case TURN_RIGHT_90: return -Math.PI / 2.0;
            case TURN_RIGHT_70_1:
            case TURN_RIGHT_70_2: return Math.toRadians(-70.0);
            case TURN_LEFT_70: return Math.toRadians(70.0);
            case TURN_RIGHT_200: return Math.toRadians(-200.0);
            default: throw new IllegalArgumentException("Not a turn: " + step);
        }
    }

    private static Pose translate(Pose from, double forward, double left) {
        double cos = cardinal(Math.cos(from.getHeading()));
        double sin = cardinal(Math.sin(from.getHeading()));
        return new Pose(from.getX() + forward * cos - left * sin,
                from.getY() + forward * sin + left * cos, from.getHeading());
    }

    private static double cardinal(double value) {
        return Math.abs(value) < 1e-12 ? 0.0 : value;
    }

    private static double normalize(double radians) {
        return (radians % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }
}
