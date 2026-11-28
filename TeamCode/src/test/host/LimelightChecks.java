package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.json.JSONObject;
import java.lang.reflect.Field;

/** Host-only regression checks. LLResult/LLResultTypes are the actual installed FTC classes. */
public final class LimelightChecks {
    private static int checks;
    private static int timestamp = 100;

    private static final class Result extends LLResult {
        long age;
        int serializations;
        String rawOverride;
        Result(String json) throws Exception { super(new JSONObject(json)); }
        @Override public long getStaleness() { return age; }
        @Override public String toString() { serializations++; return rawOverride == null ? super.toString() : rawOverride; }
    }

    private static void check(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }

    private static void near(double actual, double expected) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9,
                "Expected " + expected + ", got " + actual);
    }

    private static String tag(int id, double x, double y, double area) {
        return "{\"fID\":" + id + ",\"tx\":" + x + ",\"ty\":" + y + ",\"ta\":" + area + "}";
    }

    private static Result result(String tags) throws Exception {
        return new Result("{\"pID\":0,\"v\":1,\"ts\":" + timestamp++
                + ",\"pipelineType\":\"Fiducial\",\"Fiducial\":[" + tags + "]}");
    }

    private static void rejected(LimelightVision vision, String reason) {
        check(reason.equals(vision.getStatus()), "Expected " + reason + ", got " + vision.getStatus()
                + ": " + vision.getValidationDetails());
        check(!vision.hasTarget(), "Rejected observation must not be a target");
        check(vision.getSelectedTagId() == -1, "Rejected observation must not retain selected ID");
        check(Double.isNaN(vision.getTx()) && Double.isNaN(vision.getTy()) && Double.isNaN(vision.getTa()),
                "Rejected observation must not reuse measurements");
    }

    private static void feed(Limelight3A camera, LimelightVision vision, Result next) {
        camera.latest = next;
        int before = camera.reads;
        vision.update();
        check(camera.reads == before + 1, "Exactly one cached result read per update");
    }

    public static void main(String[] args) throws Exception {
        LimelightVision missing = new LimelightVision(new HardwareMap(null), 0, 200);
        missing.start(); missing.update();
        check(!missing.isAvailable() && !missing.hasTarget(), "Missing camera remains independent");
        missing.stop();

        Limelight3A camera = new Limelight3A();
        LimelightVision vision = new LimelightVision(new HardwareMap(camera), 0, 200);
        check(camera.switches == 0 && camera.reads == 0 && camera.starts == 0, "Constructor has no transport operations");
        camera.switchSuccess = false; vision.start();
        check(!vision.isPipelineSelected() && camera.starts == 0, "Failed pipeline selection does not start polling");
        rejected(vision, "PIPELINE_ERROR");
        camera.switchSuccess = true; vision.start();
        check(camera.pipeline == 0 && camera.starts == 1 && vision.isPipelineSelected(), "Pipeline zero selected and polling started");
        int switches = camera.switches; vision.start();
        check(camera.switches == switches && camera.starts == 1, "Repeated start is inert");
        feed(camera, vision, null); rejected(vision, "NO_RESULT");
        check(!vision.hasResult(), "Null result is explicit");
        feed(camera, vision, new Result("{}")); rejected(vision, "EMPTY_RESULT");
        check(vision.hasResult() && !vision.isResultValid(), "SDK's fabricated empty result is distinguishable from null");

        Result ordinary = result(tag(30, 3, -2, 1.25));
        feed(camera, vision, ordinary);
        check(vision.hasTarget() && "TARGET_DETECTED".equals(vision.getStatus()), "Ordinary target recognized");
        near(vision.getTx(), 3); near(vision.getTy(), -2); near(vision.getTa(), 1.25);
        check(vision.getSelectedTagId() == 30 && vision.getFiducialCount() == 1, "SDK count and selected ID exposed");
        check(vision.getResultPipelineIndex() == 0 && vision.isFresh(), "Reported pipeline and freshness exposed");
        check("Fiducial".equals(vision.getResultPipelineType()), "Reported type exposed without blocking camera status request");
        int serializations = ordinary.serializations;
        for (int i = 0; i < 10; i++) { vision.getStatus(); vision.getTx(); vision.getVisibleTagIds(); }
        feed(camera, vision, ordinary);
        check(ordinary.serializations == serializations, "Raw JSON validation occurs once per SDK result object");
        check(ordinary.getTx() == 0 && ordinary.getTy() == 0 && ordinary.getTa() == 0,
                "Actual SDK root getters fall back to zero; individual fiducials are used instead");

        feed(camera, vision, result(tag(45, 0, 0, 0)));
        check(vision.hasTarget() && vision.getSelectedTagId() == 45, "Inclusive ID45 and legitimate numeric zeros accepted");
        near(vision.getTx(), 0); near(vision.getTy(), 0); near(vision.getTa(), 0);
        feed(camera, vision, result(tag(30, .1, 10, 1) + "," + tag(31, 2, 1, 1) + "," + tag(29, 0, 0, 2)));
        check(vision.getSelectedTagId() == 31, "Nearest crosshair uses both axes and ignores disallowed ID");
        check(vision.getFiducialCount() == 3 && vision.getVisibleTagIds().contains("29"), "Diagnostics include unselected/disallowed SDK detections");
        feed(camera, vision, result(tag(29, 0, 0, 1) + "," + tag(46, 0, 0, 1)));
        rejected(vision, "NO_ALLOWED_TAG");
        feed(camera, vision, result("")); rejected(vision, "NO_FIDUCIALS");

        for (String field : new String[]{"tx", "ty", "ta"}) {
            for (Object bad : new Object[]{JSONObject.NULL, "invalid", "0", "NaN", "Infinity"}) {
                JSONObject raw = new JSONObject(tag(30, 1, 2, 3)); raw.put(field, bad);
                feed(camera, vision, result(raw.toString())); rejected(vision, "INVALID_MEASUREMENTS");
            }
            JSONObject raw = new JSONObject(tag(30, 1, 2, 3)); raw.remove(field);
            Result missingMeasurement = result(raw.toString());
            check(missingMeasurement.getFiducialResults().size() == 1, "SDK still constructs fiducials with missing fields");
            feed(camera, vision, missingMeasurement); rejected(vision, "INVALID_MEASUREMENTS");
        }
        feed(camera, vision, result(tag(30, 0, 0, -1))); rejected(vision, "INVALID_MEASUREMENTS");
        feed(camera, vision, result(tag(30, 0, 0, 101))); rejected(vision, "INVALID_MEASUREMENTS");
        feed(camera, vision, result("{\"fID\":30,\"tx\":1e999,\"ty\":0,\"ta\":1}"));
        rejected(vision, "INVALID_MEASUREMENTS");
        feed(camera, vision, result("{\"fID\":30,\"ty\":0,\"ta\":1}," + tag(31, 5, 3, 2)));
        check(vision.hasTarget() && vision.getSelectedTagId() == 31, "Malformed tag does not suppress a valid second tag");

        for (Object pipeline : new Object[]{JSONObject.NULL, "0", -1, .5}) {
            JSONObject raw = new JSONObject(result(tag(30, 0, 0, 1)).toString()); raw.put("pID", pipeline);
            feed(camera, vision, new Result(raw.toString())); rejected(vision, "INVALID_PIPELINE_METADATA");
        }
        JSONObject raw = new JSONObject(result(tag(30, 0, 0, 1)).toString()); raw.remove("pID");
        feed(camera, vision, new Result(raw.toString())); rejected(vision, "INVALID_PIPELINE_METADATA");
        check(vision.getResultPipelineIndex() == -1, "Missing metadata does not become pipeline zero");
        raw.put("pID", 1); feed(camera, vision, new Result(raw.toString())); rejected(vision, "WRONG_PIPELINE");
        check(vision.getResultPipelineIndex() == 1, "Wrong reported pipeline is visible");
        for (Object validity : new Object[]{0, JSONObject.NULL, false, "unexpected"}) {
            raw = new JSONObject(result(tag(30, 0, 0, 1)).toString()); raw.put("v", validity);
            feed(camera, vision, new Result(raw.toString())); rejected(vision, "INVALID_RESULT");
            check(!vision.isResultValid() && vision.getFiducialCount() == 1
                    && vision.getVisibleTagIds().contains("30"), "Fiducials visible diagnostically despite invalid SDK result");
        }
        raw.remove("v"); feed(camera, vision, new Result(raw.toString())); rejected(vision, "INVALID_RESULT");
        for (Object stamp : new Object[]{JSONObject.NULL, "100", 0, -1}) {
            raw = new JSONObject(result(tag(30, 0, 0, 1)).toString()); raw.put("ts", stamp);
            feed(camera, vision, new Result(raw.toString())); rejected(vision, "INVALID_TIMESTAMP");
        }

        Result stale = result(tag(30, 1, 2, 3)); stale.age = 201;
        feed(camera, vision, stale); rejected(vision, "STALE_RESULT");
        Result frozen = result(tag(30, 1, 2, 3)); feed(camera, vision, frozen);
        check(vision.hasTarget(), "New frame recovers from stale data");
        Field seen = LimelightVision.class.getDeclaredField("frameFirstSeenNanos"); seen.setAccessible(true);
        seen.setLong(vision, System.nanoTime() - 1_000_000_000L);
        // A new HTTP result object with an identical frame timestamp must not refresh capture age.
        feed(camera, vision, new Result(frozen.toString())); rejected(vision, "STALE_RESULT");
        feed(camera, vision, result(tag(32, 1, 1, 1))); check(vision.hasTarget(), "New frame recovers after frozen data");
        camera.connected = false; vision.update(); rejected(vision, "DISCONNECTED");
        camera.connected = true; feed(camera, vision, result(tag(33, 1, 1, 1)));
        check(vision.hasTarget(), "Reconnect recovers with new frame");
        vision.stop(); rejected(vision, "STOPPED");
        check(camera.stops == 1 && !vision.isRunning(), "STOP stops camera polling");
        vision.start(); vision.update(); rejected(vision, "WAITING_FOR_NEW_FRAME");
        feed(camera, vision, result(tag(34, 1, 1, 1))); check(vision.hasTarget(), "Restart waits for a fresh frame");
        System.out.println("PASS: " + checks + " Limelight checks; actual FTC 11.1 result parsers, fake transport, no physical camera.");
    }
}
