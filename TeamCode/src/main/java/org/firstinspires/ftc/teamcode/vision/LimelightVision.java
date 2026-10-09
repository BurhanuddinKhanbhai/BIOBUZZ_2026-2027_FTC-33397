package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Exposes fresh angular/area measurements from an explicitly selected Limelight pipeline. */
public class LimelightVision {
    private final Limelight3A limelight;
    private int pipelineIndex;
    private long maxResultAgeMillis;
    private boolean pipelineSelected;
    private LLResult result;
    private String status;

    private double frameTimestamp = Double.NaN;
    private long frameFirstSeenNanos;
    private long frameInitialAgeMillis;
    private boolean waitingForNewFrame = true;

    /**
     * Does not start polling or change the camera pipeline.
     * The caller must supply an existing pipeline configured to detect the hive on the camera.
     * Missing "limelight" hardware is allowed and reported through isAvailable()/getStatus().
     */
    public LimelightVision(HardwareMap hardwareMap, int pipelineIndex, long maxResultAgeMillis) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("Hardware map must not be null");
        }
        validatePipelineIndex(pipelineIndex);
        validateMaxAge(maxResultAgeMillis);
        this.pipelineIndex = pipelineIndex;
        this.maxResultAgeMillis = maxResultAgeMillis;
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        status = limelight == null ? "MISSING" : "STOPPED";
    }

    /**
     * Selects the pipeline, then starts SDK background polling. Repeated successful starts are inert.
     * Pipeline selection uses a synchronous SDK HTTP request: call during initialization, not
     * each control loop. A failed selection remains retryable by calling start() again.
     */
    public void start() {
        if (limelight == null) {
            status = "MISSING";
            return;
        }
        if (pipelineSelected && isRunning()) {
            return;
        }
        if (!selectPipeline()) {
            return;
        }
        try {
            limelight.start();
            status = limelight.isRunning() ? "NO_RESULT" : "START_ERROR";
        } catch (RuntimeException e) {
            result = null;
            pipelineSelected = false;
            status = "START_ERROR";
        }
    }

    /** Stops polling and immediately invalidates all target measurements. */
    public void stop() {
        result = null;
        pipelineSelected = false;
        waitingForNewFrame = true;
        status = limelight == null ? "MISSING" : "STOPPED";
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (RuntimeException e) {
                status = "STOP_ERROR";
            }
        }
    }

    /**
     * Reads only the SDK's cached result; no HTTP requests, waits, or image processing.
     * Call each loop. Even null/invalid results replace the previous measurements.
     */
    public void update() {
        result = null;
        if (limelight == null || !pipelineSelected || !isRunning() || !isConnected()) {
            return;
        }
        try {
            result = limelight.getLatestResult();
            status = "NO_RESULT";
            if (result == null) {
                return;
            }
            double timestamp = result.getTimestamp();
            long age = result.getStaleness();
            if (isFinite(timestamp) && timestamp > 0.0 && age >= 0 && timestamp != frameTimestamp) {
                frameTimestamp = timestamp;
                frameFirstSeenNanos = System.nanoTime();
                frameInitialAgeMillis = age;
                waitingForNewFrame = false;
            }
        } catch (RuntimeException e) {
            result = null;
            status = "READ_ERROR";
        }
    }

    /** Rechecks freshness/connection on every call, including between update() calls. */
    public boolean hasTarget() {
        return "TARGET".equals(getStatus());
    }

    /** Horizontal offset from the configured crosshair in degrees, or NaN if unavailable. */
    public double getTx() {
        return hasTarget() ? result.getTx() : Double.NaN;
    }

    /** Vertical offset from the configured crosshair in degrees, or NaN if unavailable. */
    public double getTy() {
        return hasTarget() ? result.getTy() : Double.NaN;
    }

    /** Target area as a percentage of the image (0-100), or NaN if unavailable. */
    public double getTa() {
        return hasTarget() ? result.getTa() : Double.NaN;
    }

    /**
     * Configures the requested pipeline; changing it invalidates the current result.
     * When polling, this makes a synchronous SDK HTTP request. Use outside the control loop.
     * While stopped, the pipeline is only stored and will be applied by start().
     */
    public void setPipelineIndex(int pipelineIndex) {
        validatePipelineIndex(pipelineIndex);
        if (this.pipelineIndex == pipelineIndex && pipelineSelected) {
            return;
        }
        this.pipelineIndex = pipelineIndex;
        result = null;
        pipelineSelected = false;
        waitingForNewFrame = true;
        if (isRunning()) {
            selectPipeline();
        } else {
            status = limelight == null ? "MISSING" : "STOPPED";
        }
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public void setMaxResultAgeMillis(long maxResultAgeMillis) {
        validateMaxAge(maxResultAgeMillis);
        this.maxResultAgeMillis = maxResultAgeMillis;
    }

    public long getMaxResultAgeMillis() {
        return maxResultAgeMillis;
    }

    /** Reports whether the Limelight was mapped, not whether it is currently responding. */
    public boolean isAvailable() {
        return limelight != null;
    }

    public boolean isRunning() {
        try {
            return limelight != null && limelight.isRunning();
        } catch (RuntimeException e) {
            return false;
        }
    }

    /** Uses the SDK's inexpensive recent-response connection check. */
    public boolean isConnected() {
        try {
            return limelight != null && limelight.isConnected();
        } catch (RuntimeException e) {
            return false;
        }
    }

    /**
     * Age in milliseconds, or NaN if no usable timestamp exists.
     * Combines SDK reception age with time since this camera-frame timestamp first appeared.
     * Repeated HTTP responses for a frozen frame cannot refresh that frame's age.
     * This is a freshness estimate, not a synchronized camera capture timestamp.
     */
    public double getResultAgeMillis() {
        if (result == null || waitingForNewFrame) {
            return Double.NaN;
        }
        long receivedAge = result.getStaleness();
        if (receivedAge < 0 || !isFinite(result.getTimestamp()) || result.getTimestamp() <= 0.0) {
            return Double.NaN;
        }
        double observedAge = frameInitialAgeMillis
                + (System.nanoTime() - frameFirstSeenNanos) / 1_000_000.0;
        return Math.max(receivedAge, observedAge);
    }

    /** Local telemetry status only; never calls the SDK's blocking getStatus() HTTP endpoint. */
    public String getStatus() {
        if (limelight == null) {
            return "MISSING";
        }
        if (!pipelineSelected) {
            return status;
        }
        if (!isRunning()) {
            return "START_ERROR".equals(status) ? status : "STOPPED";
        }
        if (!isConnected()) {
            return "DISCONNECTED";
        }
        if (result == null) {
            return status;
        }
        if (!result.isValid()) {
            return "NO_TARGET";
        }
        if (result.getPipelineIndex() != pipelineIndex) {
            return "PIPELINE_MISMATCH";
        }
        if (!isFinite(result.getTimestamp()) || result.getTimestamp() <= 0.0
                || !isFinite(result.getTx()) || !isFinite(result.getTy())
                || !isFinite(result.getTa()) || result.getTa() < 0.0 || result.getTa() > 100.0) {
            return "INVALID_RESULT";
        }
        if (waitingForNewFrame) {
            return "WAITING_FOR_NEW_FRAME";
        }
        double age = getResultAgeMillis();
        if (!isFinite(age) || age < 0.0) {
            return "INVALID_RESULT";
        }
        return age <= maxResultAgeMillis ? "TARGET" : "STALE";
    }

    private boolean selectPipeline() {
        result = null;
        pipelineSelected = false;
        waitingForNewFrame = true;
        try {
            if (!limelight.pipelineSwitch(pipelineIndex)) {
                status = "PIPELINE_ERROR";
                return false;
            }
            // Reject the already cached frame until a different camera timestamp arrives.
            LLResult previous = limelight.getLatestResult();
            frameTimestamp = previous == null ? Double.NaN : previous.getTimestamp();
            pipelineSelected = true;
            status = "NO_RESULT";
            return true;
        } catch (RuntimeException e) {
            status = "PIPELINE_ERROR";
            return false;
        }
    }

    private static void validatePipelineIndex(int index) {
        if (index < 0) {
            throw new IllegalArgumentException("Pipeline index must be nonnegative and exist on the camera");
        }
    }

    private static void validateMaxAge(long milliseconds) {
        if (milliseconds <= 0) {
            throw new IllegalArgumentException("Maximum result age must be positive milliseconds");
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
