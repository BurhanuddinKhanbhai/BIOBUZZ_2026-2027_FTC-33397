package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/** Exposes fresh angular/area measurements from an explicitly selected Limelight pipeline. */
public class LimelightVision {
    private final Limelight3A limelight;
    private int pipelineIndex;
    private long maxResultAgeMillis;
    private boolean pipelineSelected;
    private LLResult result;
    private String status;

    private LLResult validatedResult;
    private double targetTx = Double.NaN;
    private double targetTy = Double.NaN;
    private double targetTa = Double.NaN;
    private int resultPipelineIndex = -1;

    private int selectedTagId = -1;
    private int fiducialCount;
    private String visibleTagIds = "[]";
    private String resultPipelineType = "UNKNOWN";
    private String measurementStatus = "NO_RESULT";
    private String validationDetails = "No result";
    private double cameraTimestamp = Double.NaN;
    private boolean rawValidityConfirmed;
    private boolean emptyResult;

    /** Diagnostics describe the current cached result, even if it cannot be used for aiming. */
    public boolean hasResult() { return result != null; }
    public boolean isResultValid() { return result != null && result.isValid(); }
    public boolean isPipelineSelected() { return pipelineSelected; }
    public int getResultPipelineIndex() { return result == null ? -1 : resultPipelineIndex; }
    public String getResultPipelineType() { return result == null ? "UNKNOWN" : resultPipelineType; }
    public int getFiducialCount() { return result == null ? 0 : fiducialCount; }
    public String getVisibleTagIds() { return result == null ? "[]" : visibleTagIds; }
    public String getValidationDetails() { return result == null ? "No result" : validationDetails; }
    public double getFrameTimestamp() { return result == null ? Double.NaN : cameraTimestamp; }
    public boolean isFresh() {
        double age = getResultAgeMillis();
        return isFinite(age) && age >= 0.0 && age <= maxResultAgeMillis;
    }


    public int getSelectedTagId() {
        return hasTarget() ? selectedTagId : -1;
    }

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
        limelight.start();
        status = limelight.isRunning() ? "NO_RESULT" : "STOPPED";
    }

    /** Stops polling and immediately invalidates all target measurements. */
    public void stop() {
        result = null;
        pipelineSelected = false;
        waitingForNewFrame = true;
        status = limelight == null ? "MISSING" : "STOPPED";
        if (limelight != null) limelight.stop();
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
            validateMeasurements(result);
            double timestamp = cameraTimestamp;
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
        return "TARGET_DETECTED".equals(getStatus());
    }

    /** Horizontal offset from the configured crosshair in degrees, or NaN if unavailable. */
    public double getTx() {
        return hasTarget() ? targetTx : Double.NaN;
    }

    /** Vertical offset from the configured crosshair in degrees, or NaN if unavailable. */
    public double getTy() {
        return hasTarget() ? targetTy : Double.NaN;
    }

    /** Target area as a percentage of the image (0-100), or NaN if unavailable. */
    public double getTa() {
        return hasTarget() ? targetTa : Double.NaN;
    }

    /**
     * Configures the requested pipeline; changing it invalidates the current result.
     * When polling, this makes a synchronous SDK HTTP request. Use outside the control loop.
     * While stopped, the pipeline is only stored and will be applied by start().
     */
    public void setPipelineIndex(int pipelineIndex) {
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
        return limelight != null && limelight.isRunning();
    }

    /** Uses the SDK's inexpensive recent-response connection check. */
    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
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
        if (receivedAge < 0 || !isFinite(cameraTimestamp) || cameraTimestamp <= 0.0) {
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
            return "STOPPED";
        }
        if (!isConnected()) {
            return "DISCONNECTED";
        }
        if (result == null) {
            return status;
        }
        if (emptyResult) return "EMPTY_RESULT"; // SDK can return {} instead of null before first data.
        if (!rawValidityConfirmed || !result.isValid()) return "INVALID_RESULT";
        if (resultPipelineIndex < 0) return "INVALID_PIPELINE_METADATA";
        if (resultPipelineIndex != pipelineIndex) return "WRONG_PIPELINE";
        if (!isFinite(cameraTimestamp) || cameraTimestamp <= 0.0) return "INVALID_TIMESTAMP";
        if (waitingForNewFrame) return "WAITING_FOR_NEW_FRAME";
        double age = getResultAgeMillis();
        if (!isFinite(age) || age < 0.0) return "INVALID_TIMESTAMP";
        if (age > maxResultAgeMillis) return "STALE_RESULT";
        return measurementStatus;
    }

    /**
     * SDK 11.1.0 exposes raw JSON only through toString(); its measurement/pipeline getters
     * can substitute zero or coerce nonnumeric fields. Parse once per SDK result object,
     * including invalid results. Timestamp-based frozen-frame tracking remains separate.
     */

    private void validateMeasurements(LLResult latest) {
        if (latest == validatedResult) return;
        validatedResult = latest;
        targetTx = targetTy = targetTa = Double.NaN;
        resultPipelineIndex = selectedTagId = -1;
        cameraTimestamp = Double.NaN;
        rawValidityConfirmed = emptyResult = false;
        resultPipelineType = "UNKNOWN";
        measurementStatus = "INVALID_MEASUREMENTS";
        validationDetails = "Invalid JSON";
        // Read SDK detections even when v is false, to distinguish metadata from detection failure.
        java.util.List<LLResultTypes.FiducialResult> tags = latest.getFiducialResults();
        fiducialCount = tags == null ? 0 : tags.size();
        StringBuilder ids = new StringBuilder("[");
        if (tags != null) for (LLResultTypes.FiducialResult tag : tags) {
            if (ids.length() > 1) ids.append(", ");
            ids.append(tag.getFiducialId());
        }
        visibleTagIds = ids.append("]").toString();
        try {
            // SDK 11.1.0 toString() is the top-level /results JSON, not a Results envelope.
            JSONObject json = new JSONObject(latest.toString());
            emptyResult = json.length() == 0;
            rawValidityConfirmed = getFiniteNumber(json, "v") == 1.0;
            cameraTimestamp = getFiniteNumber(json, "ts");
            double pipeline = getFiniteNumber(json, "pID");
            if (isFinite(pipeline) && pipeline >= 0 && pipeline <= Integer.MAX_VALUE
                    && pipeline == Math.rint(pipeline)) resultPipelineIndex = (int) pipeline;
            Object type = json.opt("pipelineType");
            if (type instanceof String && !((String) type).isEmpty()) resultPipelineType = (String) type;

            JSONArray rawTags = json.optJSONArray("Fiducial");
            int allowed = 0, malformed = 0;
            double closest = Double.POSITIVE_INFINITY;
            if (rawTags != null) for (int i = 0; i < rawTags.length(); i++) {
                JSONObject tag = rawTags.optJSONObject(i);
                if (tag == null) { malformed++; continue; }
                double id = getFiniteNumber(tag, "fID");
                if (!isFinite(id) || id != Math.rint(id)) { malformed++; continue; }
                if (id < 30 || id > 45) continue;
                allowed++;
                // Fiducial SDK getters substitute zero for missing fields. Validate RAW evidence.
                double tx = getFiniteNumber(tag, "tx"), ty = getFiniteNumber(tag, "ty");
                double ta = getFiniteNumber(tag, "ta");
                if (!isFinite(tx) || !isFinite(ty) || !isFinite(ta) || ta < 0 || ta > 100) {
                    malformed++; continue;
                }
                double offset = Math.hypot(tx, ty);
                if (offset < closest) {
                    closest = offset;
                    selectedTagId = (int) id;
                    targetTx = tx; targetTy = ty; targetTa = ta;
                }
            }
            if (rawTags == null) measurementStatus = json.has("Fiducial") ? "INVALID_MEASUREMENTS" : "NO_FIDUCIALS";
            else if (rawTags.length() == 0) measurementStatus = "NO_FIDUCIALS";
            else if (selectedTagId >= 0) measurementStatus = "TARGET_DETECTED";
            else if (allowed > 0 || malformed > 0) measurementStatus = "INVALID_MEASUREMENTS";
            else measurementStatus = "NO_ALLOWED_TAG";
            validationDetails = "v=" + describe(json, "v") + " pID=" + describe(json, "pID")
                    + " ts=" + describe(json, "ts") + " root tx/ty/ta=" + describe(json, "tx")
                    + "/" + describe(json, "ty") + "/" + describe(json, "ta")
                    + " Fiducial=" + (rawTags == null ? describe(json, "Fiducial") : rawTags.length())
                    + " allowed=" + allowed + " malformed=" + malformed
                    + " candidate=" + selectedTagId + " selection=" + measurementStatus;
        } catch (JSONException e) {
            // Cached invalid result: no repeated JSON parsing or guessed measurement values.
        }
    }

    private static String describe(JSONObject json, String key) {
        if (!json.has(key)) return "MISSING";
        Object value = json.opt(key);
        if (value == null || value == JSONObject.NULL) return "NULL";
        String text = String.valueOf(value);
        return value.getClass().getSimpleName() + ":" + (text.length() > 40 ? text.substring(0, 40) : text);
    }

    private static double getFiniteNumber(JSONObject json, String key) {
        Object value = json.opt(key);
        if (!(value instanceof Number)) {
            return Double.NaN;
        }
        double number = ((Number) value).doubleValue();
        return isFinite(number) ? number : Double.NaN;
    }

    private boolean selectPipeline() {
        result = null;
        pipelineSelected = false;
        waitingForNewFrame = true;
        if (!limelight.pipelineSwitch(pipelineIndex)) {
            status = "PIPELINE_ERROR";
            return false;
        }
        // A cached result from the previous pipeline is not a new camera observation.
        LLResult previous = limelight.getLatestResult();
        frameTimestamp = previous == null ? Double.NaN : previous.getTimestamp();
        pipelineSelected = true;
        status = "NO_RESULT";
        return true;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
