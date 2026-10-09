package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

/**
 * Coordinates sensor observations and shooter readiness without operating a feeder.
 * The transfer feeds directly into the shooter: software readiness does not confirm safe
 * physical ball staging. This class never commands transfer, release, or firing hardware.
 */
public class ShootingCoordinator {
    private final Shooter shooter;
    private final BallColorSensor colorSensor;
    private final LimelightVision vision;
    private boolean readyToFeed;

    /** Allows operation without vision; a null color sensor reports UNKNOWN. */
    public ShootingCoordinator(Shooter shooter, BallColorSensor colorSensor) {
        this(shooter, colorSensor, null);
    }

    /**
     * Requires an existing shooter; either sensor wrapper may be null or have missing hardware.
     * Construction leaves shooter mode and hardware commands unchanged.
     * The caller configures calibration, velocity, and selection mode on Shooter directly;
     * call Shooter.setAutoMode() to enable automatic selection. If supplied, configure and
     * start/stop LimelightVision outside this coordinator's control-loop update.
     */
    public ShootingCoordinator(Shooter shooter, BallColorSensor colorSensor,
                               LimelightVision vision) {
        if (shooter == null) {
            throw new IllegalArgumentException("Shooter must not be null");
        }
        this.shooter = shooter;
        this.colorSensor = colorSensor;
        this.vision = vision;
    }

    /**
     * Call once per control loop, after any caller-issued shooter commands.
     * Passes the current filtered classification, including UNKNOWN, to Shooter, which
     * preserves manual overrides. Readiness uses Shooter's existing checks; vision is
     * informational and never changes velocity or gates readiness. No waits or delays.
     */
    public void update() {
        readyToFeed = false;
        if (colorSensor != null) {
            colorSensor.update();
        }
        shooter.update(getDetectedBallType());
        if (vision != null) {
            vision.update();
        }
        readyToFeed = shooter.isReadyToFeed();
    }

    /**
     * Software readiness from the latest completed update(), initially false.
     * Changes to shooter commands or sensor conditions require another update().
     * This snapshot does not establish safe physical staging or trigger feeding.
     */
    public boolean isReadyToFeed() {
        return readyToFeed;
    }

    /** Overall software status from the latest update(), initially NOT_READY. */
    public String getStatus() {
        return readyToFeed ? "SOFTWARE_READY" : "NOT_READY";
    }

    /** Latest filtered sensor classification; distinct from a manual shooter selection. */
    public BallColorSensor.BallType getDetectedBallType() {
        return colorSensor == null ? BallColorSensor.BallType.UNKNOWN : colorSensor.getBallType();
    }

    public Shooter.BallType getSelectedBallType() {
        return shooter.getSelectedBallType();
    }

    public Shooter.SelectionMode getSelectionMode() {
        return shooter.getSelectionMode();
    }

    /** NOT_CONFIGURED means no wrapper was supplied; MISSING is reported by the wrapper. */
    public String getColorSensorStatus() {
        return colorSensor == null ? "NOT_CONFIGURED" : colorSensor.getStatus();
    }

    /** Uses LimelightVision's current freshness/connection checks; false without vision. */
    public boolean hasVisionTarget() {
        return vision != null && vision.hasTarget();
    }

    /** NOT_CONFIGURED means no wrapper was supplied; otherwise returns its local status. */
    public String getVisionStatus() {
        return vision == null ? "NOT_CONFIGURED" : vision.getStatus();
    }
}
