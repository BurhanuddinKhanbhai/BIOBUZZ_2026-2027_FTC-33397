package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.BallColorCalibration;
import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;

/** Mounted color sampling, explicit calibration saving, and hold-A servo testing. */
@TeleOp(name = "TEST - Fast Color", group = "Testing")
public class FastColorTest extends OpMode {
    // Preserve this test OpMode's presets; competition TeleOp has separate calibrated presets.
    private static final double POLLEN_POSITION = 0.30;
    private static final double NECTAR_POSITION = 0.02;

    private BallColorSensor detector;
    private Servo adjust;
    private final BallColorCalibration.Samples empty = new BallColorCalibration.Samples();
    private final BallColorCalibration.Samples pollen = new BallColorCalibration.Samples();
    private final BallColorCalibration.Samples nectar = new BallColorCalibration.Samples();
    private final BallColorCalibration.Samples redNectar = new BallColorCalibration.Samples();
    private BallColorCalibration savedCalibration;
    private boolean emptyTouched, pollenTouched, nectarTouched, redTouched;
    private boolean sessionReady;
    private BallColorSensor.BallType lastCommanded = BallColorSensor.BallType.UNKNOWN;
    private boolean previousSave;
    private String sessionStatus = "COLLECT EMPTY / YELLOW / BLUE; RED OPTIONAL";

    @Override
    public void init() {
        detector = new BallColorSensor(hardwareMap);
        // Retain loaded references. Only explicitly sampled groups are replaced on SAVE.
        savedCalibration = detector.getCalibration();
        sessionReady = savedCalibration != null;
        if (sessionReady) sessionStatus = "LOADED - unchanged groups will be preserved";
        adjust = hardwareMap.tryGet(Servo.class, "shooterAdjust");
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("X=EMPTY, Y=yellow POLLEN, B=blue NECTAR (hold each separately, 12+ samples)");
        telemetry.addLine("LEFT BUMPER=red NECTAR (12+ valid samples); existing saved groups are retained.");
        telemetry.addLine("Release sample buttons, then press D-pad UP to SAVE. Incomplete groups block save.");
        telemetry.addLine("Hold A to allow servo commands after calibration. No movement during init.");
        telemetry.update();
    }

    @Override
    public void start() {
        previousSave = gamepad1.dpad_up;
    }

    @Override
    public void loop() {
        detector.update(); // One shared color read and at most one distance read.
        boolean captureEmpty = gamepad1.x && !gamepad1.y && !gamepad1.b && !gamepad1.left_bumper;
        boolean capturePollen = gamepad1.y && !gamepad1.x && !gamepad1.b && !gamepad1.left_bumper;
        boolean captureNectar = gamepad1.b && !gamepad1.x && !gamepad1.y && !gamepad1.left_bumper;
        boolean captureRed = gamepad1.left_bumper && !gamepad1.x && !gamepad1.y && !gamepad1.b;
        boolean calibrating = gamepad1.x || gamepad1.y || gamepad1.b || gamepad1.left_bumper;
        if (calibrating) {
            emptyTouched |= captureEmpty;
            pollenTouched |= capturePollen;
            nectarTouched |= captureNectar;
            redTouched |= captureRed;
            if (captureEmpty && detector.hasColorSample()) {
                // Preserve the local EMPTY fix: dark samples count as zero fractions.
                empty.add(detector.hasValidReading() ? detector.getRedFraction() : 0.0,
                        detector.hasValidReading() ? detector.getGreenFraction() : 0.0,
                        detector.hasValidReading() ? detector.getBlueFraction() : 0.0,
                        detector.getDistanceCm());
            } else if (detector.hasValidReading()) {
                if (capturePollen) addSample(pollen);
                if (captureNectar) addSample(nectar);
                if (captureRed) addSample(redNectar);
            }
            if (captureEmpty || capturePollen || captureNectar || captureRed) {
                refreshCalibration();
            } else {
                sessionStatus = "CONFLICT - hold only one sample button";
            }
        }

        boolean save = gamepad1.dpad_up;
        if (save && !previousSave) {
            if (calibrating) {
                sessionStatus = "Release sample buttons before saving";
            } else if (!sessionReady) {
                sessionStatus = "NOT SAVED - complete/validate every newly sampled group (12+ each)";
            } else if (detector.saveCalibration()) {
                savedCalibration = detector.getCalibration();
                sessionStatus = "SAVED - unchanged profiles retained; competition TeleOp loads on init";
            } else {
                sessionStatus = detector.getCalibrationStatus();
            }
        }
        previousSave = save;
        BallColorSensor.BallType detected = detector.getBallType();
        if (!calibrating && gamepad1.a && adjust != null
                && detected != BallColorSensor.BallType.UNKNOWN && detected != lastCommanded) {
            adjust.setPosition(detected == BallColorSensor.BallType.POLLEN ? POLLEN_POSITION : NECTAR_POSITION);
            lastCommanded = detected;
        }

        telemetry.addData("Session", sessionStatus);
        telemetry.addData("Calibration", detector.getCalibrationStatus());
        telemetry.addData("Status", detector.getStatus());
        telemetry.addData("Color mapped", detector.isAvailable());
        telemetry.addData("Shooter servo mapped", adjust != null);
        telemetry.addData("New samples E / Y / B / R", "%d / %d / %d / %d",
                empty.getCount(), pollen.getCount(), nectar.getCount(), redNectar.getCount());
        telemetry.addData("Red nectar calibration", detector.isRedCalibrated() ? "CONFIGURED" : "UNCALIBRATED (yellow/blue still available)");
        telemetry.addLine("X=empty, Y=yellow, B=blue, LB=red; release then D-pad UP=save; A=servo test.");
        telemetry.addData("RGB fraction R/G/B", "%.3f / %.3f / %.3f",
                detector.getRedFraction(), detector.getGreenFraction(), detector.getBlueFraction());
        telemetry.addData("Distance / gate (cm)", "%.2f / %.2f (NaN = color-only)",
                detector.getDistanceCm(), detector.getGateDistanceCm());
        telemetry.addData("Candidate / confirmed", "%s / %s", detector.getCandidateBallType(), detected);
        telemetry.addData("Confirm readings", detector.getConsecutiveReadings());
        telemetry.addData("Read / confirmation (ms)", "%.2f / %.2f", detector.getReadTimeMs(), detector.getConfirmationTimeMs());
        telemetry.addData("Hold A to command servo", gamepad1.a);
        telemetry.addData("Last servo command", lastCommanded);
        telemetry.addLine("Timing excludes physical ball arrival and servo travel. Blue and red both select NECTAR.");
        telemetry.update();
    }

    private void addSample(BallColorCalibration.Samples samples) {
        samples.add(detector.getRedFraction(), detector.getGreenFraction(), detector.getBlueFraction(), detector.getDistanceCm());
    }

    private void refreshCalibration() {
        sessionReady = false;
        try {
            BallColorCalibration next = new BallColorCalibration(
                    replacement(empty, emptyTouched, savedCalibration == null ? null : savedCalibration.empty),
                    replacement(pollen, pollenTouched, savedCalibration == null ? null : savedCalibration.pollen),
                    replacement(nectar, nectarTouched, savedCalibration == null ? null : savedCalibration.nectar),
                    replacement(redNectar, redTouched, savedCalibration == null ? null : savedCalibration.redNectar));
            detector.configureCalibration(next);
            sessionReady = true;
            sessionStatus = "READY TO SAVE - release sample buttons; untouched profiles retained";
        } catch (IllegalStateException | IllegalArgumentException ex) {
            // Keep the working detector and saved file intact, but do not save partial replacements.
            sessionStatus = "NOT READY: " + ex.getMessage();
        }
    }

    private static BallColorCalibration.Reference replacement(BallColorCalibration.Samples samples,
                                                              boolean touched, BallColorCalibration.Reference saved) {
        return touched ? samples.average() : saved;
    }
}
