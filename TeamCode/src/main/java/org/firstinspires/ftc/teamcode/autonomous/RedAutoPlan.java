package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

/** Robot-relative Red autonomous route. Edit the route values together in Step below. */
public final class RedAutoPlan {

    /*
     * ===================== AUTONOMOUS STEP SETTINGS =====================
     * Everything in the route is listed here, IN EXECUTION ORDER.
     *
     * Each row: STEP (forward inches, left inches, turn degrees, max power, finish tolerance, seconds)
     *
     * Forward: + forward, - backward (relative to the robot at that step).
     * Left:    + robot-left, - robot-right (relative to the robot at that step).
     * Turn:    + counterclockwise/left, - clockwise/right.
     * Power:   0.0 to 1.0 maximum drivetrain power (driving/turning steps).
     * Finish tolerance: INCHES on drives; DEGREES on turns.
     * Seconds: feeding duration on SHOOT steps; waiting time on FLOWER_DOWN.
     *
     * IMPORTANT: INTAKE_FLOWER and PARK_FORWARD are 0/0 placeholders.
     * Set their forward and left offsets once measured. Zero-length drives are skipped.
     * Only change numbers below; do not reorder/remove enum entries without updating the controller.
     * ===================================================================
     */
    public enum Step {
        // STEP                 FORWARD   LEFT    TURN°   POWER  TOLERANCE  SECONDS

        CLEAR_WALL_FORWARD (  4.0,   0.0,   0.0,  0.35,  0.5,  0.0),
        SHOOT_1            (  0.0,   0.0,   0.0,  0.00,  0.0,  3.0),
        TURN_LEFT_90       (  0.0,   0.0, 180.0,  1.00,  5.0,  0.0),
        CLEAR_WALL_STRAFE  (-5.0, -46.0,   0.0,  1.0,  3.0,  0.0),
        INTAKE_1           ( 14.0,   0.0,   0.0,  0.75,  8.0,  2.0),

        RETURN_1          (-15.0, 26.0,   0.0, 1.00, 4.0, 0.0),
        DRIVE_TO_SHOOT_2  (-100.0, 0.0,   0.0, 1.00, 1.0, 0.0),
        TURN_RIGHT_70_1   (  0.0,  0.0,  40.0, 1.00, 5.0, 0.0),
        SHOOT_2           (  0.0,  0.0,   0.0, 0.00, 0.0, 4.0),

        FLOWER_DOWN        (  0.0, 0.0,    0.0, 0.00, 0.0, 1.0),

        TURN_TO_ZERO       (0.0, 0.0, 130.0, 1.00, 5.0, 0.0),
        INTAKE_FLOWER      (  5.0, -6.0,    0.0, 0.75, 0.5, 3.0),
        RETURN_FROM_FLOWER ( -5.0, 0.0,    0.0, 0.75, 0.5, 0.0),
        TURN_RIGHT_150_2   (  0.0, 0.0, -150.0, 1.00, 5.0, 0.0),
        SHOOT_3            (  0.0, 0.0,    0.0, 0.00, 0.0, 4.0),
        PARK_BACKWARD      (5.0, -10.0,    0.0, 1.00, 1.0, 0.0);

        private final double forwardInches;
        private final double leftInches;
        private final double turnDegrees;
        private final double maxPower;
        private final double finishTolerance;
        private final double durationSeconds;

        Step(double forwardInches, double leftInches, double turnDegrees,
             double maxPower, double finishTolerance, double durationSeconds) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
            this.turnDegrees = turnDegrees;
            this.maxPower = maxPower;
            this.finishTolerance = finishTolerance;
            this.durationSeconds = durationSeconds;
        }

        public double getForwardInches() { return forwardInches; }
        public double getLeftInches() { return leftInches; }
        public double getTurnDegrees() { return turnDegrees; }
        public double getMaxPower() { return maxPower; }
        public double getFinishTolerance() { return finishTolerance; }
        public double getDurationSeconds() { return durationSeconds; }
    }

    private final Pose start = new Pose(0, 0, 0);
    private final Pose[] targets = new Pose[Step.values().length];

    public RedAutoPlan() {
        Pose planned = start;
        for (Step step : Step.values()) {
            if (isTurn(step)) {
                planned = new Pose(planned.getX(), planned.getY(),
                        normalize(planned.getHeading() + turnRadians(step)));
            } else if (isDrive(step)) {
                planned = translate(planned, step.getForwardInches(), step.getLeftInches());
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
                || step == Step.RETURN_FROM_FLOWER
                || step == Step.PARK_BACKWARD;
    }



    public static boolean isTurn(Step step) {
        return step == Step.TURN_LEFT_90
                || step == Step.TURN_RIGHT_70_1
                || step == Step.TURN_TO_ZERO
                || step == Step.TURN_RIGHT_150_2;
    }



    public static boolean isShoot(Step step) {
        return step == Step.SHOOT_1 || step == Step.SHOOT_2 || step == Step.SHOOT_3;
    }

    public static double turnRadians(Step step) {
        if (!isTurn(step)) throw new IllegalArgumentException("Not a turn: " + step);
        return Math.toRadians(step.getTurnDegrees());
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
