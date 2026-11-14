package org.firstinspires.ftc.teamcode.sensors;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.zip.CRC32;

/** Mounted REV V3 references for EMPTY, yellow pollen, blue nectar, and optional red nectar. */
public final class BallColorCalibration {
    public static final int VERSION = 2;
    public static final int REQUIRED_SAMPLES = 12;
    public static final int CONFIRM_READINGS = 2;
    public static final double MIN_COLOR_SIGNAL = 0.003;
    public static final double MAX_COLOR_DISTANCE = 0.13;
    public static final double MIN_MARGIN = 0.035;
    public static final String FILE_NAME = "biobuzz-ball-color.bin";
    private static final int MAGIC = 0x42424331; // Retained so existing v1 files remain readable.
    private static final int V1_PAYLOAD_BYTES = 8 + 3 * (4 * 8 + 2 * 4);
    // V2 adds an explicit 0/1 presence byte followed by a fourth reference only when calibrated.
    private static final int MAX_PAYLOAD_BYTES = V1_PAYLOAD_BYTES + 1 + (4 * 8 + 2 * 4);

    public static final class Reference {
        public final double red, green, blue, distanceCm;
        public final int samples, distanceSamples;

        public Reference(double red, double green, double blue, double distanceCm,
                         int samples, int distanceSamples) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.distanceCm = distanceCm;
            this.samples = samples;
            this.distanceSamples = distanceSamples;
        }

        public double error(double r, double g, double b) {
            double dr = r - red, dg = g - green, db = b - blue;
            return Math.sqrt(dr * dr + dg * dg + db * db);
        }
    }

    /** A fresh calibration session; zero/dark EMPTY readings must still count. */
    public static final class Samples {
        private double red, green, blue, distance;
        private int count, distanceCount;

        public void add(double r, double g, double b, double cm) {
            if (!fraction(r) || !fraction(g) || !fraction(b) || r + g + b > 1.00001
                    || count == Integer.MAX_VALUE) {
                throw new IllegalArgumentException("Invalid calibration sample");
            }
            red += r;
            green += g;
            blue += b;
            count++;
            if (validDistance(cm)) {
                distance += cm;
                distanceCount++;
            }
        }

        public int getCount() { return count; }
        public boolean isReady() { return count >= REQUIRED_SAMPLES; }

        public Reference average() {
            if (!isReady()) throw new IllegalStateException("Collect at least 12 samples per group");
            return new Reference(red / count, green / count, blue / count,
                    distanceCount == 0 ? Double.NaN : distance / distanceCount, count, distanceCount);
        }
    }

    /** nectar retains its original meaning: the BLUE reference. */
    public final Reference empty, pollen, nectar, redNectar;
    private final int formatVersion;

    public BallColorCalibration(Reference empty, Reference pollen, Reference nectar) {
        this(empty, pollen, nectar, null);
    }

    public BallColorCalibration(Reference empty, Reference pollen, Reference nectar, Reference redNectar) {
        this(empty, pollen, nectar, redNectar, VERSION);
    }

    private BallColorCalibration(Reference empty, Reference pollen, Reference nectar,
                                 Reference redNectar, int formatVersion) {
        validate(empty, true);
        validate(pollen, false);
        validate(nectar, false);
        if (empty.error(pollen.red, pollen.green, pollen.blue) <= MIN_MARGIN
                || empty.error(nectar.red, nectar.green, nectar.blue) <= MIN_MARGIN
                || pollen.error(nectar.red, nectar.green, nectar.blue) <= MIN_MARGIN) {
            throw new IllegalArgumentException("Calibration references are too similar; collect distinct samples");
        }
        if (redNectar != null) {
            validate(redNectar, false); // Includes the same minimum of 12 valid samples.
            // Red and blue both mean NECTAR, so they need not be distinguishable from each other.
            if (empty.error(redNectar.red, redNectar.green, redNectar.blue) <= MIN_MARGIN
                    || pollen.error(redNectar.red, redNectar.green, redNectar.blue) <= MIN_MARGIN) {
                throw new IllegalArgumentException("Red nectar must be distinguishable from EMPTY and POLLEN");
            }
        }
        this.empty = empty;
        this.pollen = pollen;
        this.nectar = nectar;
        this.redNectar = redNectar;
        this.formatVersion = formatVersion;
    }

    public int getFormatVersion() { return formatVersion; }

    public double getGateDistanceCm() {
        return gateDistance(nectar);
    }

    /** Red's optional/unreliable proximity data must not weaken the working yellow/blue gate. */
    public double getRedGateDistanceCm() {
        return redNectar == null ? Double.NaN : gateDistance(redNectar);
    }

    private double gateDistance(Reference nectarReference) {
        if (!validDistance(empty.distanceCm) || !validDistance(pollen.distanceCm)
                || !validDistance(nectarReference.distanceCm)) return Double.NaN;
        double farthestBall = Math.max(pollen.distanceCm, nectarReference.distanceCm);
        return empty.distanceCm - farthestBall >= 0.5
                ? (empty.distanceCm + farthestBall) / 2.0 : Double.NaN;
    }

    public static boolean validDistance(double cm) {
        return Double.isFinite(cm) && cm > 0.0 && cm < 30.0;
    }

    /** Load a complete, versioned, checksummed file. Never salvage a partial calibration. */
    public static BallColorCalibration load(File file) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream(MAX_PAYLOAD_BYTES + 8);
        try (FileInputStream in = new FileInputStream(file)) {
            int value;
            while ((value = in.read()) != -1) {
                if (bytes.size() >= MAX_PAYLOAD_BYTES + 8) throw new IOException("Unexpected calibration data");
                bytes.write(value);
            }
        }
        if (bytes.size() < V1_PAYLOAD_BYTES + 8) throw new IOException("Incomplete calibration data");
        byte[] payload = new byte[bytes.size() - 8];
        long checksum;
        try (DataInputStream in = new DataInputStream(new ByteArrayInputStream(bytes.toByteArray()))) {
            in.readFully(payload);
            checksum = in.readLong();
            if (in.read() != -1) throw new IOException("Unexpected calibration data");
        }
        CRC32 crc = new CRC32();
        crc.update(payload);
        if (crc.getValue() != checksum) throw new IOException("Calibration checksum mismatch");
        try (DataInputStream in = new DataInputStream(new ByteArrayInputStream(payload))) {
            int magic = in.readInt();
            int version = in.readInt();
            if (magic != MAGIC || (version != 1 && version != VERSION)) {
                throw new IOException("Unsupported calibration format/version");
            }
            try {
                Reference empty = readReference(in), pollen = readReference(in), nectar = readReference(in);
                Reference red = null;
                if (version == VERSION) {
                    int hasRed = in.readUnsignedByte();
                    if (hasRed > 1) throw new IOException("Invalid red calibration presence flag");
                    if (hasRed == 1) red = readReference(in);
                }
                if (in.read() != -1) throw new IOException("Unexpected calibration fields");
                return new BallColorCalibration(empty, pollen, nectar, red, version);
            } catch (IllegalArgumentException ex) {
                throw new IOException("Invalid calibration values", ex);
            }
        }
    }

    /**
     * Explicit action only, never in update(). Sync a complete temporary file, then rename
     * within the app's files directory. Android/Linux rename replaces atomically; never
     * delete the previous file first. A failed/interrupted write cannot expose partial data.
     */
    public void save(File file) throws IOException {
        if (file.exists() && !file.isFile()) throw new IOException("Calibration destination is not a regular file");
        ByteArrayOutputStream bytes = new ByteArrayOutputStream(MAX_PAYLOAD_BYTES);
        try (DataOutputStream out = new DataOutputStream(bytes)) {
            out.writeInt(MAGIC);
            out.writeInt(VERSION);
            writeReference(out, empty);
            writeReference(out, pollen);
            writeReference(out, nectar);
            out.writeByte(redNectar == null ? 0 : 1);
            if (redNectar != null) writeReference(out, redNectar);
        }
        byte[] payload = bytes.toByteArray();
        CRC32 crc = new CRC32();
        crc.update(payload);
        File temporary = File.createTempFile("ball-color-", ".tmp", file.getAbsoluteFile().getParentFile());
        try {
            try (FileOutputStream stream = new FileOutputStream(temporary);
                 DataOutputStream out = new DataOutputStream(stream)) {
                out.write(payload);
                out.writeLong(crc.getValue());
                out.flush();
                stream.getFD().sync();
            }
            if (!temporary.renameTo(file)) throw new IOException("Could not atomically replace calibration file");
        } finally {
            // An orphan temporary file is harmless: loading only ever uses FILE_NAME.
            if (temporary.exists()) temporary.delete();
        }
    }

    private static Reference readReference(DataInputStream in) throws IOException {
        return new Reference(in.readDouble(), in.readDouble(), in.readDouble(), in.readDouble(),
                in.readInt(), in.readInt());
    }

    private static void writeReference(DataOutputStream out, Reference reference) throws IOException {
        out.writeDouble(reference.red);
        out.writeDouble(reference.green);
        out.writeDouble(reference.blue);
        out.writeDouble(reference.distanceCm);
        out.writeInt(reference.samples);
        out.writeInt(reference.distanceSamples);
    }

    private static void validate(Reference r, boolean empty) {
        if (r == null || !fraction(r.red) || !fraction(r.green) || !fraction(r.blue)
                || r.samples < REQUIRED_SAMPLES || r.distanceSamples < 0 || r.distanceSamples > r.samples) {
            throw new IllegalArgumentException("Incomplete or invalid calibration reference");
        }
        double sum = r.red + r.green + r.blue;
        // EMPTY can average dark (0,0,0) and illuminated samples; its sum need not be one.
        if (sum > 1.00001 || (!empty && Math.abs(sum - 1.0) > 0.00001)
                || (r.distanceSamples == 0 ? !Double.isNaN(r.distanceCm) : !validDistance(r.distanceCm))) {
            throw new IllegalArgumentException("Invalid normalized RGB or proximity reference");
        }
    }

    private static boolean fraction(double value) {
        return Double.isFinite(value) && value >= 0.0 && value <= 1.0;
    }
}
