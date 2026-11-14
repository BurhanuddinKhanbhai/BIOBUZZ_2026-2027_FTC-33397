package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.IOException;

/** Shared fast REV V3 detector; the explicit raw-profile constructor remains for legacy callers. */
public class BallColorSensor {
    public enum BallType { POLLEN, NECTAR, UNKNOWN }

    /** Inclusive raw RGB bounds, measured using the installed sensor and actual game pieces. */
    public static final class ColorProfile {
        private final int minRed;
        private final int maxRed;
        private final int minGreen;
        private final int maxGreen;
        private final int minBlue;
        private final int maxBlue;

        public ColorProfile(int minRed, int maxRed, int minGreen, int maxGreen,
                            int minBlue, int maxBlue) {
            validateRange(minRed, maxRed);
            validateRange(minGreen, maxGreen);
            validateRange(minBlue, maxBlue);
            this.minRed = minRed;
            this.maxRed = maxRed;
            this.minGreen = minGreen;
            this.maxGreen = maxGreen;
            this.minBlue = minBlue;
            this.maxBlue = maxBlue;
        }

        private boolean matches(int red, int green, int blue) {
            return red >= minRed && red <= maxRed
                    && green >= minGreen && green <= maxGreen
                    && blue >= minBlue && blue <= maxBlue;
        }

        private static void validateRange(int min, int max) {
            if (min < 0 || max < min) {
                throw new IllegalArgumentException("Color bounds must be nonnegative with min <= max");
            }
        }
    }

    private final ColorSensor sensor;
    private final boolean normalizedMode;
    private final NormalizedColorSensor normalizedSensor;
    private final DistanceSensor proximity;
    private final File calibrationFile;
    private BallColorCalibration calibration;
    private String calibrationStatus = "UNCONFIGURED";
    private double redFraction = Double.NaN, greenFraction = Double.NaN, blueFraction = Double.NaN;
    private double distanceCm = Double.NaN;
    private double readTimeMs = Double.NaN, confirmationTimeMs = Double.NaN;
    private double bestColorError = Double.NaN, colorMargin = Double.NaN;
    private boolean colorSampleAvailable;
    private boolean goodNormalizedColor;
    private boolean matchedRedNectar;
    private int consecutiveReadings;
    private ColorProfile pollenProfile;
    private ColorProfile nectarProfile;
    private long minimumSignal = -1;
    private double stableTimeSeconds;
    private int red = -1;
    private int green = -1;
    private int blue = -1;
    private int alpha = -1;
    private long signal = -1;
    private BallType ballType = BallType.UNKNOWN;
    private BallType candidateBallType = BallType.UNKNOWN;
    private long candidateSinceNanos;
    private String status;

    /**
     * A missing "ballColor" device is allowed: isAvailable() is false and classification UNKNOWN.
     * Supply a positive stability interval in seconds; no color calibration is assumed.
     */
    public BallColorSensor(HardwareMap hardwareMap, double stableTimeSeconds) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("Hardware map must not be null");
        }
        validateStableTime(stableTimeSeconds);
        this.stableTimeSeconds = stableTimeSeconds;
        normalizedMode = false;
        normalizedSensor = null;
        proximity = null;
        calibrationFile = null;
        sensor = hardwareMap.tryGet(ColorSensor.class, "ballColor");
        status = sensor == null ? "MISSING" : "NOT_UPDATED";
    }

    /** Production path: one normalized read per update, persistent calibration, two-reading confirmation. */
    public BallColorSensor(HardwareMap hardwareMap) {
        this(hardwareMap.tryGet(NormalizedColorSensor.class, "ballColor"),
                new File(hardwareMap.appContext.getFilesDir(), BallColorCalibration.FILE_NAME));
    }

    /** Explicit dependencies for software tests. A missing sensor is allowed; no hardware reads in construction. */
    public BallColorSensor(NormalizedColorSensor sensor, File calibrationFile) {
        if (calibrationFile == null) throw new IllegalArgumentException("Calibration file is required");
        this.sensor = null;
        normalizedMode = true;
        normalizedSensor = sensor;
        proximity = sensor instanceof DistanceSensor ? (DistanceSensor) sensor : null;
        this.calibrationFile = calibrationFile;
        loadCalibration();
    }

    public void configureCalibration(BallColorCalibration calibration) {
        if (!normalizedMode || calibration == null) {
            throw new IllegalArgumentException("Normalized sensor and complete calibration required");
        }
        this.calibration = calibration;
        calibrationStatus = "IN_MEMORY_UNSAVED";
        clearClassification(isAvailable() ? "NOT_UPDATED" : "MISSING");
    }

    /** Clears this session only; never deletes a previously saved calibration. */
    public void clearCalibration() {
        calibration = null;
        pollenProfile = nectarProfile = null;
        minimumSignal = -1;
        calibrationStatus = "UNCALIBRATED";
        clearClassification(isAvailable() ? "UNCALIBRATED" : "MISSING");
    }

    /** Initialization/explicit action only; no file access occurs in update(). */
    public boolean loadCalibration() {
        if (!normalizedMode) throw new IllegalStateException("Persistent references require normalized mode");
        clearCalibration();
        try {
            if (!calibrationFile.exists()) {
                calibrationStatus = "MISSING - run Fast Color calibration";
                return false;
            }
            calibration = BallColorCalibration.load(calibrationFile);
            calibrationStatus = "LOADED v" + calibration.getFormatVersion() + calibrationColors();
            clearClassification(isAvailable() ? "NOT_UPDATED" : "MISSING");
            return true;
        } catch (IOException | RuntimeException ex) {
            calibrationStatus = "INVALID - " + ex.getMessage();
            return false;
        }
    }

    /** Explicit operator save only. Incomplete groups cannot produce a calibration object. */
    public boolean saveCalibration() {
        if (!normalizedMode || calibration == null) {
            calibrationStatus = "NOT_SAVED - complete EMPTY/yellow/blue calibration";
            return false;
        }
        try {
            calibration.save(calibrationFile);
            calibrationStatus = "SAVED v" + BallColorCalibration.VERSION + calibrationColors();
            return true;
        } catch (IOException | RuntimeException ex) {
            calibrationStatus = "SAVE_FAILED - " + ex.getMessage();
            return false;
        }
    }

    /**
     * Configures both profiles and a nonnegative minimum raw RGB sum, then clears classification.
     * The signal threshold is a brightness gate, not a physical distance measurement.
     * Overlapping profile matches are always UNKNOWN. Zero RGB signal is always too weak.
     */
    public void configureCalibration(ColorProfile pollen, ColorProfile nectar, long minimumSignal) {
        if (normalizedMode || pollen == null || nectar == null || minimumSignal < 0) {
            throw new IllegalArgumentException("Both color profiles and a nonnegative signal threshold are required");
        }
        pollenProfile = pollen;
        nectarProfile = nectar;
        this.minimumSignal = minimumSignal;
        clearClassification(sensor == null ? "MISSING" : "NOT_UPDATED");
    }

    /**
     * Call once each control loop. Reads each raw channel once and filters unique matches.
     * Missing, invalid, weak, ambiguous, or unmatched readings immediately clear classification.
     * A changed candidate reports UNKNOWN until it has been observed for the stability interval.
     * Filtering does not block or sleep; readings between calls cannot be observed.
     */
    public void update() {
        if (normalizedMode) {
            updateNormalized();
            return;
        }
        if (sensor == null) {
            clearReadings();
            clearClassification("MISSING");
            return;
        }

        try {
            int nextRed = sensor.red();
            int nextGreen = sensor.green();
            int nextBlue = sensor.blue();
            int nextAlpha = sensor.alpha();
            red = nextRed;
            green = nextGreen;
            blue = nextBlue;
            alpha = nextAlpha;
        } catch (RuntimeException e) {
            clearReadings();
            clearClassification("READ_ERROR");
            return;
        }

        if (!hasValidReading()) {
            signal = -1;
            clearClassification("INVALID_READING");
            return;
        }
        signal = (long) red + green + blue;
        if (!isCalibrated()) {
            clearClassification("UNCALIBRATED");
            return;
        }
        if (signal == 0 || signal < minimumSignal) {
            clearClassification("WEAK_SIGNAL");
            return;
        }

        boolean matchesPollen = pollenProfile.matches(red, green, blue);
        boolean matchesNectar = nectarProfile.matches(red, green, blue);
        if (matchesPollen && matchesNectar) {
            clearClassification("AMBIGUOUS");
            return;
        }
        if (!matchesPollen && !matchesNectar) {
            clearClassification("NO_MATCH");
            return;
        }

        BallType candidate = matchesPollen ? BallType.POLLEN : BallType.NECTAR;
        long now = System.nanoTime();
        if (candidate != candidateBallType) {
            candidateBallType = candidate;
            candidateSinceNanos = now;
            ballType = BallType.UNKNOWN;
        }
        if ((now - candidateSinceNanos) / 1_000_000_000.0 >= stableTimeSeconds) {
            ballType = candidate;
            status = "CLASSIFIED";
        } else {
            status = "STABILIZING";
        }
    }

    /** Returns the filtered result from the most recent update(). */
    public BallType getBallType() {
        return ballType;
    }

    /** Diagnostic candidate only; it must not be treated as a stable classification. */
    public BallType getCandidateBallType() {
        return candidateBallType;
    }

    /** Reports whether a compatible device was mapped, not whether its latest read succeeded. */
    public boolean isAvailable() {
        return normalizedMode ? normalizedSensor != null : sensor != null;
    }

    public boolean isCalibrated() {
        return normalizedMode ? calibration != null
                : pollenProfile != null && nectarProfile != null && minimumSignal >= 0;
    }

    /** Numeric reading validity only; a valid reading does not establish ball presence. */
    public boolean hasValidReading() {
        return normalizedMode ? goodNormalizedColor : red >= 0 && green >= 0 && blue >= 0 && alpha >= 0;
    }

    public String getStatus() {
        return status;
    }

    /** Raw red reading; -1 before a complete read or after a read error. */
    public int getRed() {
        return red;
    }

    /** Raw green reading; -1 before a complete read or after a read error. */
    public int getGreen() {
        return green;
    }

    /** Raw blue reading; -1 before a complete read or after a read error. */
    public int getBlue() {
        return blue;
    }

    /** Raw alpha reading for telemetry only; never interpreted as physical distance. */
    public int getAlpha() {
        return alpha;
    }

    /** Raw RGB sum, or -1 when no valid reading is available. */
    public long getSignal() {
        return signal;
    }

    /** Returns the configured RGB-sum threshold, or -1 while uncalibrated. */
    public long getMinimumSignal() {
        return minimumSignal;
    }

    /** Legacy raw-profile timing only; normalized detection always uses two consecutive readings. */
    public void setStableTimeSeconds(double stableTimeSeconds) {
        if (normalizedMode) throw new IllegalStateException("Normalized detection uses reading confirmation, not a time delay");
        validateStableTime(stableTimeSeconds);
        if (this.stableTimeSeconds != stableTimeSeconds) {
            this.stableTimeSeconds = stableTimeSeconds;
            clearClassification(sensor == null ? "MISSING" : "NOT_UPDATED");
        }
    }

    public double getStableTimeSeconds() {
        return stableTimeSeconds;
    }

    private void clearClassification(String status) {
        ballType = BallType.UNKNOWN;
        candidateBallType = BallType.UNKNOWN;
        candidateSinceNanos = 0;
        consecutiveReadings = 0;
        confirmationTimeMs = Double.NaN;
        this.status = status;
    }

    private void clearReadings() {
        red = green = blue = alpha = -1;
        signal = -1;
        redFraction = greenFraction = blueFraction = distanceCm = Double.NaN;
        bestColorError = colorMargin = Double.NaN;
        colorSampleAvailable = goodNormalizedColor = false;
        matchedRedNectar = false;
    }

    /** Cached telemetry only. Legacy raw-channel getters return -1 in normalized mode. */
    public double getRedFraction() { return redFraction; }
    public double getGreenFraction() { return greenFraction; }
    public double getBlueFraction() { return blueFraction; }
    public double getDistanceCm() { return distanceCm; }
    public double getGateDistanceCm() {
        return calibration == null ? Double.NaN : matchedRedNectar
                ? calibration.getRedGateDistanceCm() : calibration.getGateDistanceCm();
    }
    /** Immutable references, for extending saved calibration without resampling unchanged groups. */
    public BallColorCalibration getCalibration() { return calibration; }
    public boolean isRedCalibrated() { return calibration != null && calibration.redNectar != null; }
    public String getCalibrationStatus() { return calibrationStatus; }
    public int getConsecutiveReadings() { return consecutiveReadings; }
    public double getBestColorError() { return bestColorError; }
    public double getColorMargin() { return colorMargin; }
    public double getReadTimeMs() { return readTimeMs; }
    /** First confident candidate to confirmation, excluding physical arrival and servo travel. */
    public double getConfirmationTimeMs() { return confirmationTimeMs; }
    /** A finite nonnegative sample exists, even if too dark to normalize (needed for EMPTY capture). */
    public boolean hasColorSample() { return colorSampleAvailable; }

    private void updateNormalized() {
        clearReadings();
        readTimeMs = Double.NaN;
        if (normalizedSensor == null) {
            clearClassification("MISSING");
            return;
        }
        NormalizedRGBA rgba;
        long readStarted = System.nanoTime();
        try {
            rgba = normalizedSensor.getNormalizedColors();
            distanceCm = proximity == null ? Double.NaN : proximity.getDistance(DistanceUnit.CM);
        } catch (RuntimeException ex) {
            clearReadings();
            clearClassification("READ_ERROR");
            return;
        } finally {
            readTimeMs = (System.nanoTime() - readStarted) / 1_000_000.0;
        }
        if (rgba == null || !Float.isFinite(rgba.red) || !Float.isFinite(rgba.green)
                || !Float.isFinite(rgba.blue) || !Float.isFinite(rgba.alpha)
                || rgba.red < 0 || rgba.green < 0 || rgba.blue < 0 || rgba.alpha < 0) {
            clearClassification("INVALID_COLOR");
            return;
        }
        colorSampleAvailable = true;
        double sum = (double) rgba.red + rgba.green + rgba.blue;
        goodNormalizedColor = sum > BallColorCalibration.MIN_COLOR_SIGNAL;
        if (!goodNormalizedColor) {
            clearClassification("WEAK_SIGNAL");
            return;
        }
        redFraction = rgba.red / sum;
        greenFraction = rgba.green / sum;
        blueFraction = rgba.blue / sum;
        if (calibration == null) {
            clearClassification("UNCALIBRATED");
            return;
        }

        double e = calibration.empty.error(redFraction, greenFraction, blueFraction);
        double p = calibration.pollen.error(redFraction, greenFraction, blueFraction);
        double blueError = calibration.nectar.error(redFraction, greenFraction, blueFraction);
        double redError = calibration.redNectar == null ? Double.POSITIVE_INFINITY
                : calibration.redNectar.error(redFraction, greenFraction, blueFraction);
        // Compare ball TYPES, not the two same-type nectar colors, for the confidence margin.
        double n = Math.min(blueError, redError);
        matchedRedNectar = redError < blueError && n < p;
        bestColorError = Math.min(p, n);
        colorMargin = Math.min(e, Math.max(p, n)) - bestColorError;
        BallType next = BallType.UNKNOWN;
        if (p < BallColorCalibration.MAX_COLOR_DISTANCE && e - p > BallColorCalibration.MIN_MARGIN
                && n - p > BallColorCalibration.MIN_MARGIN) {
            next = BallType.POLLEN;
        } else if (n < BallColorCalibration.MAX_COLOR_DISTANCE && e - n > BallColorCalibration.MIN_MARGIN
                && p - n > BallColorCalibration.MIN_MARGIN) {
            next = BallType.NECTAR;
        }
        double gate = getGateDistanceCm();
        // Preserve the tested fallback: unreliable distance uses confident color matching only.
        if (Double.isFinite(gate) && BallColorCalibration.validDistance(distanceCm) && distanceCm > gate) {
            clearClassification("NO_OBJECT_NEAR");
            return;
        }
        if (next == BallType.UNKNOWN) {
            clearClassification("NO_CONFIDENT_MATCH");
            return;
        }
        long now = System.nanoTime();
        if (next != candidateBallType) {
            clearClassification("CLASSIFYING");
            candidateBallType = next;
            candidateSinceNanos = now;
        }
        if (consecutiveReadings < BallColorCalibration.CONFIRM_READINGS) consecutiveReadings++;
        if (consecutiveReadings >= BallColorCalibration.CONFIRM_READINGS) {
            if (ballType != next) confirmationTimeMs = (now - candidateSinceNanos) / 1_000_000.0;
            ballType = next;
            status = "CLASSIFIED";
        } else {
            status = "CLASSIFYING";
        }
    }

    private String calibrationColors() {
        return isRedCalibrated() ? " (yellow / blue / red)" : " (yellow / blue; red UNCALIBRATED)";
    }

    private static void validateStableTime(double seconds) {
        if (Double.isNaN(seconds) || Double.isInfinite(seconds) || seconds <= 0.0) {
            throw new IllegalArgumentException("Color stability time must be finite and positive");
        }
    }
}
