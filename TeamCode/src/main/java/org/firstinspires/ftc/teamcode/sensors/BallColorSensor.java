package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Classifies raw color readings using explicitly calibrated ranges and stability filtering. */
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
        sensor = hardwareMap.tryGet(ColorSensor.class, "ballColor");
        status = sensor == null ? "MISSING" : "NOT_UPDATED";
    }

    /**
     * Configures both profiles and a nonnegative minimum raw RGB sum, then clears classification.
     * The signal threshold is a brightness gate, not a physical distance measurement.
     * Overlapping profile matches are always UNKNOWN. Zero RGB signal is always too weak.
     */
    public void configureCalibration(ColorProfile pollen, ColorProfile nectar, long minimumSignal) {
        if (pollen == null || nectar == null || minimumSignal < 0) {
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
        return sensor != null;
    }

    public boolean isCalibrated() {
        return pollenProfile != null && nectarProfile != null && minimumSignal >= 0;
    }

    /** Numeric reading validity only; a valid reading does not establish ball presence. */
    public boolean hasValidReading() {
        return red >= 0 && green >= 0 && blue >= 0 && alpha >= 0;
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

    /** A changed positive, finite interval clears the current classification. */
    public void setStableTimeSeconds(double stableTimeSeconds) {
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
        this.status = status;
    }

    private void clearReadings() {
        red = green = blue = alpha = -1;
        signal = -1;
    }

    private static void validateStableTime(double seconds) {
        if (Double.isNaN(seconds) || Double.isInfinite(seconds) || seconds <= 0.0) {
            throw new IllegalArgumentException("Color stability time must be finite and positive");
        }
    }
}
