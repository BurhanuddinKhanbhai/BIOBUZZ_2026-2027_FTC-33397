package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

/**
 * The requested route, in Pedro inches: origin at the diagram's bottom-left,
 * +X right, +Y up, heading zero right, positive heading counterclockwise.
 * The intake/shooter starts pointing left (PI). No hardware is accessed here.
 *
 * Clearance is a measured circle about the localization origin enclosing the
 * entire robot, including the lowered flower arm, throughout every rotation.
 * A back-flat-against-wall start may not have sufficient corner-sweep clearance;
 * this plan rejects it rather than silently shifting or shortening the route.
 * Rectangular field containment does not verify clearance from game structures.
 */
public final class RedAutoPlan {
    public enum Step {
        SHOOT_1, TURN_LEFT_90, INTAKE_1, RETURN_1, TURN_RIGHT_90,
        DRIVE_TO_SHOOT_2, TURN_180_1, SHOOT_2, FLOWER_DOWN, TURN_180_2,
        BACK_HALF_TILE_1, STRAFE_LEFT, INTAKE_2, BACK_HALF_TILE_2, TURN_180_3, SHOOT_3
    }

    private final Pose start;
    private final Pose[] targets = new Pose[Step.values().length];
    private final double fieldWidth;
    private final double fieldHeight;
    private final double inset;

    public RedAutoPlan(double startX, double startY, double intake2Distance,
                       double fieldWidth, double fieldHeight,
                       double robotClearanceRadius, double margin) {
        requireFinite(startX, "starting X");
        requireFinite(startY, "starting Y");
        requirePositive(intake2Distance, "INTAKE_2 distance");
        requirePositive(fieldWidth, "verified field width");
        requirePositive(fieldHeight, "verified field height");
        requirePositive(robotClearanceRadius, "measured swept robot clearance radius");
        requireFinite(margin, "field clearance margin");
        if (margin < 0.0) throw new IllegalArgumentException("Field clearance margin must be nonnegative");
        inset = robotClearanceRadius + margin;
        if (!Double.isFinite(inset) || fieldWidth <= 2.0 * inset || fieldHeight <= 2.0 * inset) {
            throw new IllegalArgumentException("Verified field cannot contain the swept robot footprint and margin");
        }
        this.fieldWidth = fieldWidth;
        this.fieldHeight = fieldHeight;
        start = new Pose(startX, startY, Math.PI);
        requireInside(start, "START (including first turn's swept clearance)");
        Pose planned = start;
        for (Step step : Step.values()) {
            if (isTurn(step)) {
                planned = new Pose(planned.getX(), planned.getY(),
                        normalize(planned.getHeading() + turnRadians(step)));
            } else {
                switch (step) {
                    case INTAKE_1: planned = translate(planned, 48.0, 0.0); break;
                    case RETURN_1: planned = translate(planned, -72.0, 0.0); break;
                    case DRIVE_TO_SHOOT_2: planned = translate(planned, 120.0, 0.0); break;
                    case BACK_HALF_TILE_1:
                    case BACK_HALF_TILE_2: planned = translate(planned, -12.0, 0.0); break;
                    case STRAFE_LEFT: planned = translate(planned, 0.0, 12.0); break;
                    case INTAKE_2: planned = translate(planned, intake2Distance, 0.0); break;
                    default: break; // Shooting and flower commands retain the preceding planned pose.
                }
            }
            // The inset rectangle is convex: both endpoints inside guarantee the straight
            // segment lies inside. The circular footprint also covers the complete turn sweep.
            requireInside(planned, (step.ordinal() + 1) + " " + step);
            targets[step.ordinal()] = planned;
        }
    }

    public Pose getStart() { return start; }

    public Pose getTarget(Step step) {
        if (step == null) throw new IllegalArgumentException("Autonomous step is required");
        return targets[step.ordinal()];
    }

    public boolean contains(Pose pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getHeading())
                && pose.getX() >= inset && pose.getX() <= fieldWidth - inset
                && pose.getY() >= inset && pose.getY() <= fieldHeight - inset;
    }

    public static boolean isDrive(Step step) {
        return step == Step.INTAKE_1 || step == Step.RETURN_1 || step == Step.DRIVE_TO_SHOOT_2
                || step == Step.BACK_HALF_TILE_1 || step == Step.STRAFE_LEFT
                || step == Step.INTAKE_2 || step == Step.BACK_HALF_TILE_2;
    }

    public static boolean isTurn(Step step) {
        return step == Step.TURN_LEFT_90 || step == Step.TURN_RIGHT_90
                || step == Step.TURN_180_1 || step == Step.TURN_180_2 || step == Step.TURN_180_3;
    }

    public static boolean isShoot(Step step) {
        return step == Step.SHOOT_1 || step == Step.SHOOT_2 || step == Step.SHOOT_3;
    }

    /**
     * Signed planned heading change. Half-turn targets use +PI; Pedro's heading controller
     * may take either shortest arc to that target. The requested half-turns have no fixed direction.
     */
    public static double turnRadians(Step step) {
        if (step == Step.TURN_LEFT_90) return Math.PI / 2.0;
        if (step == Step.TURN_RIGHT_90) return -Math.PI / 2.0;
        if (step == Step.TURN_180_1 || step == Step.TURN_180_2 || step == Step.TURN_180_3) return Math.PI;
        throw new IllegalArgumentException("Not a turn step: " + step);
    }

    private static Pose translate(Pose from, double forward, double left) {
        // All specified headings are cardinal. Remove trigonometric round-off at an
        // exact field boundary, without relaxing the physical containment limits.
        double cos = cardinal(Math.cos(from.getHeading()));
        double sin = cardinal(Math.sin(from.getHeading()));
        return new Pose(from.getX() + forward * cos - left * sin,
                from.getY() + forward * sin + left * cos, from.getHeading());
    }

    private void requireInside(Pose pose, String label) {
        if (!contains(pose)) {
            throw new IllegalArgumentException(label + " is outside verified field clearance: X="
                    + pose.getX() + ", Y=" + pose.getY() + "; allowed X=" + inset + ".."
                    + (fieldWidth - inset) + ", Y=" + inset + ".." + (fieldHeight - inset)
                    + ". Verify route/start/robot dimensions; no path was shortened or shifted.");
        }
    }

    private static double cardinal(double value) { return Math.abs(value) < 1e-12 ? 0.0 : value; }
    private static double normalize(double radians) { return (radians % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI); }
    private static void requirePositive(double value, String name) {
        requireFinite(value, name);
        if (value <= 0.0) throw new IllegalArgumentException(name + " must be positive");
    }
    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be explicitly configured and finite");
    }
}
