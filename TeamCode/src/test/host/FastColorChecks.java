package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FastColorTest;
import org.firstinspires.ftc.teamcode.ServoCalibration;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teleOp.Teleop;

import java.io.*;
import java.lang.reflect.*;
import java.nio.file.Files;
import java.util.*;
import java.util.zip.CRC32;

/** Run with src/test/run_fast_color_checks.py; synthetic references are NOT robot calibration. */
public final class FastColorChecks {
    private static int checks;
    private static void check(boolean condition, String reason) {
        checks++;
        if (!condition) throw new AssertionError(reason);
    }
    private static void near(double a, double b) { check(Math.abs(a - b) < 1e-6, a + " != " + b); }
    private static Object field(Object target, String name) throws Exception {
        Field f = target.getClass().getDeclaredField(name); f.setAccessible(true); return f.get(target);
    }
    private static BallColorCalibration calibration() {
        return new BallColorCalibration(reference(0, 0, 0, 12), reference(.3, .6, .1, 3), reference(.1, .2, .7, 4));
    }
    private static BallColorCalibration withRed() {
        BallColorCalibration base = calibration();
        return new BallColorCalibration(base.empty, base.pollen, base.nectar, reference(.75, .15, .10, 4));
    }
    /** Reproduce the original v1 format independently of the current serializer. */
    private static byte[] legacyCalibration() throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        try (DataOutputStream out = new DataOutputStream(bytes)) {
            out.writeInt(0x42424331); out.writeInt(1);
            BallColorCalibration base = calibration();
            for (BallColorCalibration.Reference r : new BallColorCalibration.Reference[]{base.empty, base.pollen, base.nectar}) {
                out.writeDouble(r.red); out.writeDouble(r.green); out.writeDouble(r.blue); out.writeDouble(r.distanceCm);
                out.writeInt(r.samples); out.writeInt(r.distanceSamples);
            }
        }
        byte[] payload = bytes.toByteArray();
        CRC32 crc = new CRC32(); crc.update(payload);
        try (DataOutputStream out = new DataOutputStream(bytes)) { out.writeLong(crc.getValue()); }
        return bytes.toByteArray();
    }
    private static void writeChecksummed(File file, byte[] bytes) throws IOException {
        CRC32 crc = new CRC32(); crc.update(bytes, 0, bytes.length - 8);
        try (DataOutputStream out = new DataOutputStream(new FileOutputStream(file))) {
            out.write(bytes, 0, bytes.length - 8); out.writeLong(crc.getValue());
        }
    }
    private static void sameReference(BallColorCalibration.Reference a, BallColorCalibration.Reference b) {
        check(a.red == b.red && a.green == b.green && a.blue == b.blue
                && Double.compare(a.distanceCm, b.distanceCm) == 0
                && a.samples == b.samples && a.distanceSamples == b.distanceSamples, "Unchanged reference preserved exactly");
    }
    private static BallColorCalibration.Reference reference(double r, double g, double b, double d) {
        return new BallColorCalibration.Reference(r, g, b, d, 12, Double.isNaN(d) ? 0 : 12);
    }
    private static void expectFailure(Runnable operation) {
        boolean rejected = false;
        try { operation.run(); } catch (IllegalArgumentException | IllegalStateException expected) { rejected = true; }
        check(rejected, "Expected argument/state failure");
    }
    private static final class Color {
        float r = .3f, g = .6f, b = .1f;
        double distance = 3;
        int reads, distanceReads;
        boolean failed;
        final NormalizedColorSensor device = (NormalizedColorSensor) Proxy.newProxyInstance(
                NormalizedColorSensor.class.getClassLoader(),
                new Class<?>[]{NormalizedColorSensor.class, DistanceSensor.class}, (proxy, method, args) -> {
                    if (method.getName().equals("getNormalizedColors")) {
                        reads++;
                        if (failed) throw new IllegalStateException("disconnected");
                        NormalizedRGBA rgba = new NormalizedRGBA();
                        rgba.red = r; rgba.green = g; rgba.blue = b; rgba.alpha = 1;
                        return rgba;
                    }
                    if (method.getName().equals("getDistance")) { distanceReads++; return distance; }
                    throw new AssertionError("Unexpected sensor operation: " + method.getName());
                });
        void rgb(float r, float g, float b) { this.r = r; this.g = g; this.b = b; }
    }
    private static final class Motor {
        double power, velocity;
        DcMotorSimple.Direction direction;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower": power = (double) args[0]; return null;
                        case "setVelocity": velocity = (double) args[0]; return null;
                        case "getVelocity": return velocity;
                        case "setDirection": direction = (DcMotorSimple.Direction) args[0]; return null;
                        case "setMode": check(args[0] != DcMotor.RunMode.STOP_AND_RESET_ENCODER, "Encoder reset"); return null;
                        case "setZeroPowerBehavior": return null;
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
    }
    private static final class Adjustment {
        int calls, attempts, directionCalls, lastLoop;
        boolean failPosition;
        Servo.Direction direction;
        double position = Double.NaN;
        final Servo device = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("setDirection")) {
                        direction = (Servo.Direction) args[0]; directionCalls++; return null;
                    }
                    if (method.getName().equals("setPosition")) {
                        attempts++;
                        if (failPosition) throw new IllegalStateException("simulated servo write failure");
                        position = (double) args[0]; calls++; lastLoop = flowerLoopNumber; return null;
                    }
                    throw new AssertionError("Unexpected servo operation: " + method.getName());
                });
    }
    private static HardwareMap hardware(File directory, Color color, Adjustment adjustment, Map<String, Motor> motors) {
        HardwareMap map = new HardwareMap(directory);
        if (color != null) map.put("ballColor", color.device);
        map.put("shooterAdjust", adjustment.device);
        map.put("flowerServo", new Adjustment().device);
        map.put("flowerServo2", new Adjustment().device);
        for (String name : new String[]{"frontLeft", "frontRight", "backLeft", "backRight", "intake", "transfer", "flywheelA", "flywheelB"}) {
            Motor motor = new Motor(); motors.put(name, motor); map.put(name, motor.device);
        }
        return map;
    }
    private static void initialize(OpMode op, HardwareMap map) {
        op.hardwareMap = map;
        op.telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> method.getReturnType() == boolean.class ? false : null);
        op.init(); op.start();
    }
    private static void rb(Teleop op) { op.gamepad2.right_bumper = true; op.loop(); op.gamepad2.right_bumper = false; op.loop(); }

    private static int flowerLoopNumber;
    private static void flowerLoop(Teleop op) { flowerLoopNumber++; op.loop(); }

    private static void dualFlowerChecks(File directory) throws Exception {
        Adjustment first = new Adjustment(), second = new Adjustment();
        HardwareMap map = new HardwareMap(directory);
        map.put("flowerServo", first.device); map.put("flowerServo2", second.device);
        FlowerMechanism linked = new FlowerMechanism(map, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        check(first.calls == 0 && second.calls == 0, "Linked constructor never commands positions");
        check(first.direction == Servo.Direction.FORWARD && second.direction == Servo.Direction.REVERSE
                && first.directionCalls == 1 && second.directionCalls == 1, "Servo directions applied exactly once");
        check(linked.isServo1Available() && linked.isServo2Available(), "Both linked servos mapped");
        check(Double.isNaN(linked.getTargetPosition1()) && Double.isNaN(linked.getTargetPosition2()), "Uncommanded targets are unknown");
        linked.raise(); linked.lower();
        check(first.calls == 0 && second.calls == 0, "Unset endpoints do not invent a target");
        linked.setPosition(.4);
        near(first.position, .4); near(second.position, .4);
        check(!linked.arePositionsConfigured(), "Direct target needs no calibration arming step");
        int directCalls = first.calls;
        linked.configurePositions(.20, .70);
        check(linked.arePositionsConfigured() && linked.isReady(), "Configured endpoints available");
        check(first.calls == directCalls && second.calls == directCalls, "Configuring endpoints does not move servos");
        linked.raise(); near(first.position, .20); near(second.position, .20);
        near(linked.getTargetPosition1(), .20); near(linked.getTargetPosition2(), .20); near(linked.getTargetPosition(), .20);
        linked.lower(); near(first.position, .70); near(second.position, .70);
        near(linked.getTargetPosition1(), .70); near(linked.getTargetPosition2(), .70);
        check(first.directionCalls == 1 && second.directionCalls == 1, "Commands never reapply or invert directions");

        // A failed SDK write propagates and does not create a custom latch requiring re-arming.
        for (int failedSide = 0; failedSide < 2; failedSide++) {
            Adjustment one = new Adjustment(), two = new Adjustment();
            HardwareMap failureMap = new HardwareMap(directory);
            failureMap.put("flowerServo", one.device); failureMap.put("flowerServo2", two.device);
            FlowerMechanism faulty = new FlowerMechanism(failureMap, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
            faulty.configurePositions(.20, .70);
            (failedSide == 0 ? one : two).failPosition = true;
            expectFailure(faulty::lower);
            if (failedSide == 0) check(Double.isNaN(faulty.getTargetPosition1()) && two.attempts == 0, "First SDK failure propagates before the second write");
            else near(faulty.getTargetPosition1(), .70);
            check(Double.isNaN(faulty.getTargetPosition2()), "Failed write never appears as successful target");
            one.failPosition = two.failPosition = false;
            faulty.raise();
            near(one.position, .20); near(two.position, .20);
            check(faulty.getState().equals("RAISE_COMMANDED"), "Next explicit command needs no fault reset");
        }

        // Production TeleOp: same-loop commands, rising edges, conflicts and independent controls.
        first = new Adjustment(); second = new Adjustment();
        Map<String, Motor> motors = new HashMap<>();
        map = hardware(directory, null, new Adjustment(), motors);
        map.put("flowerServo", first.device); map.put("flowerServo2", second.device);
        Teleop op = new Teleop(); op.gamepad2.dpad_right = true;
        initialize(op, map); flowerLoop(op);
        check(first.calls == 0 && second.calls == 0, "No INIT/START movement or held-at-START action");
        op.gamepad2.dpad_right = false; flowerLoop(op);
        op.gamepad1.left_stick_y = -1; op.gamepad2.x = true; op.gamepad2.left_trigger = .3f;
        op.gamepad2.dpad_right = true; flowerLoop(op);
        near(first.position, .3); near(second.position, .3);
        check(first.calls == 1 && second.calls == 1 && first.lastLoop == flowerLoopNumber
                && second.lastLoop == flowerLoopNumber, "UP writes both targets in one control loop");
        near(motors.get("frontLeft").power, 1); near(motors.get("frontRight").power, 1);
        near(motors.get("intake").power, .6); near(motors.get("transfer").power, .15);
        flowerLoop(op); check(first.calls == 1 && second.calls == 1, "Holding RIGHT sends no repeated command");
        op.gamepad1.left_stick_y = 0; op.gamepad2.x = false; op.gamepad2.left_trigger = 0;
        op.gamepad2.dpad_right = false; flowerLoop(op);
        op.gamepad2.dpad_left = true; flowerLoop(op);
        near(first.position, .5); near(second.position, .5);
        check(first.calls == 2 && second.calls == 2 && first.lastLoop == flowerLoopNumber
                && second.lastLoop == flowerLoopNumber, "DOWN writes both targets in one control loop");
        flowerLoop(op); check(first.calls == 2 && second.calls == 2, "Holding LEFT sends no repeated command");
        op.gamepad2.dpad_left = false; flowerLoop(op);
        op.gamepad2.dpad_left = op.gamepad2.dpad_right = true; flowerLoop(op);
        check(first.calls == 2 && second.calls == 2, "Conflicting directions are ignored");
        op.gamepad2.dpad_left = false; flowerLoop(op);
        check(first.calls == 2 && second.calls == 2, "Releasing conflict does not trigger held direction");
        op.gamepad2.dpad_right = false; flowerLoop(op); op.gamepad2.dpad_right = true; flowerLoop(op);
        check(first.calls == 3 && second.calls == 3, "New press after conflict works once");
        check(first.direction == Servo.Direction.FORWARD && second.direction == Servo.Direction.REVERSE
                && first.directionCalls == 1 && second.directionCalls == 1, "TeleOp reverses servo 2 exactly once");
        op.stop(); check(first.calls == 3 && second.calls == 3, "STOP never moves the flower arm");
        for (Motor motor : motors.values()) { near(motor.power, 0); near(motor.velocity, 0); }

        // Required linked hardware is resolved through the SDK's normal initialization error.
        for (String absent : new String[]{"flowerServo", "flowerServo2"}) {
            HardwareMap missingMap = hardware(directory, null, new Adjustment(), new HashMap<>());
            missingMap.put(absent, null);
            expectFailure(() -> initialize(new Teleop(), missingMap));
        }
    }

    private static void servoCalibrationChecks(File directory) {
        Adjustment first = new Adjustment(), second = new Adjustment(), shooter = new Adjustment();
        HardwareMap map = new HardwareMap(directory);
        map.put("flowerServo", first.device); map.put("flowerServo2", second.device);
        map.put("shooterAdjust", shooter.device);
        ServoCalibration.flowerPosition = .20;
        ServoCalibration.flower2Position = .20;
        ServoCalibration.shooterPosition = .30;
        ServoCalibration.flowerEnabled = ServoCalibration.flower2Enabled = ServoCalibration.shooterEnabled = true;
        ServoCalibration op = new ServoCalibration();
        initialize(op, map);
        check(first.calls == 0 && second.calls == 0 && shooter.calls == 0, "Calibration INIT/START sends no position commands");
        check(second.direction == Servo.Direction.REVERSE && second.directionCalls == 1, "Calibration reverses servo 2 once");
        op.loop();
        near(first.position, .20); near(second.position, .20); near(shooter.position, .30);
        check(first.calls == 1 && second.calls == 1 && shooter.calls == 1, "Enabled targets apply without OFF-ON arming");
        op.loop(); check(first.calls == 1 && second.calls == 1 && shooter.calls == 1, "Unchanged targets are not rewritten");
        ServoCalibration.flowerPosition = ServoCalibration.flower2Position = .70;
        ServoCalibration.shooterPosition = .02;
        op.loop();
        near(first.position, .70); near(second.position, .70); near(shooter.position, .02);
        check(first.calls == 2 && second.calls == 2 && shooter.calls == 2, "Full target change applies in one loop without rate limiting");
        ServoCalibration.flowerEnabled = false; ServoCalibration.flowerPosition = .30;
        op.loop(); near(first.position, .70);
        ServoCalibration.flowerEnabled = true; op.loop(); near(first.position, .30);
        int total = first.calls + second.calls + shooter.calls;
        op.stop();
        check(!ServoCalibration.flowerEnabled && !ServoCalibration.flower2Enabled && !ServoCalibration.shooterEnabled,
                "STOP clears normal enable controls");
        check(total == first.calls + second.calls + shooter.calls, "STOP sends no servo position");
        // Restore UI defaults after host fixtures.
        ServoCalibration.flowerPosition = ServoCalibration.flower2Position = ServoCalibration.shooterPosition = .50;
    }

    private static void competitionControlChecks(File directory) throws Exception {
        Map<String, Motor> motors = new HashMap<>();
        Teleop op = new Teleop();
        initialize(op, hardware(directory, null, new Adjustment(), motors));
        // Exhaust the effective local hold-control combinations, including LT's unchanged no-op.
        for (int combination = 0; combination < 32; combination++) {
            op.gamepad2.a = (combination & 1) != 0;
            op.gamepad2.b = (combination & 2) != 0;
            op.gamepad2.x = (combination & 4) != 0;
            op.gamepad2.left_trigger = (combination & 8) != 0 ? .3f : 0;
            op.gamepad2.right_trigger = (combination & 16) != 0 ? .3f : 0;
            op.loop();
            double intake = op.gamepad2.right_trigger > .2 ? -.50
                    : op.gamepad2.a || op.gamepad2.b ? 1 : op.gamepad2.x ? .6 : 0;
            double transfer = op.gamepad2.right_trigger > .2 ? -.50
                    : op.gamepad2.a ? 1.0 : op.gamepad2.b ? 0 : op.gamepad2.x ? .15 : 0;
            near(motors.get("intake").power, intake); near(motors.get("transfer").power, transfer);
        }
        op.gamepad2.a = op.gamepad2.b = op.gamepad2.x = false;
        op.gamepad2.left_trigger = op.gamepad2.right_trigger = 0;
        op.gamepad1.left_stick_y = -.01f; op.loop();
        near(motors.get("frontLeft").power, 0);
        op.gamepad1.left_stick_y = -1; op.gamepad1.right_bumper = true; op.loop();
        near(motors.get("frontLeft").power, 1); near(motors.get("frontRight").power, 1);
        op.gamepad1.left_bumper = true; op.loop();
        near(motors.get("frontLeft").power, .6); near(motors.get("frontRight").power, .6);
        op.gamepad1.left_stick_x = op.gamepad1.right_stick_x = 1; op.loop();
        for (String name : new String[]{"frontLeft", "frontRight", "backLeft", "backRight"}) {
            check(Math.abs(motors.get(name).power) <= .6, "Mecanum normalization and precision scale retained");
        }
        op.stop();
        for (Motor motor : motors.values()) { near(motor.power, 0); near(motor.velocity, 0); }

        // Optional adjustment hardware still permits flywheel-only controls and manual intent.
        motors = new HashMap<>();
        HardwareMap map = hardware(directory, null, new Adjustment(), motors);
        map.put("shooterAdjust", null);
        op = new Teleop(); initialize(op, map); rb(op);
        Shooter shooter = (Shooter) field(op, "shooter");
        check(shooter.getSelectionMode() == Shooter.SelectionMode.MANUAL_POLLEN
                && shooter.getSelectedBallType() == Shooter.BallType.UNKNOWN, "Missing optional servo keeps manual intent without false applied target");
        op.gamepad2.y = op.gamepad2.left_bumper = op.gamepad2.dpad_up = true; op.loop();
        near(motors.get("flywheelA").velocity, 50); near(motors.get("flywheelB").velocity, 50);
        op.gamepad2.y = op.gamepad2.left_bumper = op.gamepad2.dpad_up = false; op.loop();
        op.gamepad2.left_bumper = true; op.loop();
        near(motors.get("flywheelA").velocity, 0); near(motors.get("flywheelB").velocity, 0);
        check(!shooter.isReadyToFeed(), "AUTO speed has no invented target and missing servo cannot report feed ready");
        op.stop();
    }

    private static double visionMath(String name, double... args) throws Exception {
        Class<?>[] types = new Class<?>[args.length]; java.util.Arrays.fill(types, double.class);
        java.lang.reflect.Method method = Teleop.class.getDeclaredMethod(name, types); method.setAccessible(true);
        Object[] boxed = new Object[args.length]; for(int i=0;i<args.length;i++) boxed[i]=args[i];
        return (double) method.invoke(null, boxed);
    }

    private static void visionChecks(File directory) throws Exception {
        near(visionMath("estimateDistance", 15, 10, 30, 30), 20);
        for (double[] invalid : new double[][]{{Double.NaN,10,30,30},{0,10,30,0},{0,10,10,30},
                {0,10,30,90},{0,10,30,225},{0,10,30,-30},{0,-10,30,30}})
            check(Double.isNaN(visionMath("estimateDistance",invalid)), "Invalid geometry rejected");
        near(visionMath("interpolatedSpeed",60,40,80,1010,1190),1100);
        near(visionMath("interpolatedSpeed",10,40,80,1010,1190),1010);
        near(visionMath("interpolatedSpeed",100,40,80,1010,1190),1190);
        near(visionMath("stabilizedSpeed",1010,Double.NaN,1010,1190),1010);
        near(visionMath("stabilizedSpeed",1190,Double.NaN,1010,1190),1190);
        near(visionMath("stabilizedSpeed",1140,1100,1010,1190),1100);
        near(visionMath("stabilizedSpeed",1190,1100,1010,1190),1190);
        check(Double.isNaN(visionMath("interpolatedSpeed",60,40,40,1000,1200)),"Invalid calibration rejected");
        check(Double.isNaN(visionMath("interpolatedSpeed",60,-1,80,1000,1200)),"Negative calibration distance rejected");
        Map<String,Motor> motors=new HashMap<>(); Teleop op=new Teleop();
        initialize(op, hardware(directory,null,new Adjustment(),motors));
        org.firstinspires.ftc.teamcode.vision.LimelightVision vision =
                (org.firstinspires.ftc.teamcode.vision.LimelightVision)field(op,"limelight");
        op.gamepad1.right_bumper=true; op.gamepad1.right_stick_x=.2f;
        vision.tx=20; vision.ty=10; op.loop();
        near(motors.get("frontLeft").power,.35); near(motors.get("frontRight").power,-.35);
        check(vision.updates==1,"Exactly one vision update per loop");
        vision.tx=-20; op.loop(); near(motors.get("frontLeft").power,-.35);
        vision.tx=1; op.loop(); near(motors.get("frontLeft").power,0);
        vision.tx=Double.NaN; op.loop(); near(motors.get("frontLeft").power,(double).2f);
        vision.tx=20; vision.ty=Double.NaN; op.loop(); near(motors.get("frontLeft").power,(double).2f);
        vision.ty=10; op.gamepad1.right_bumper=false; op.loop(); near(motors.get("frontLeft").power,(double).2f);
        op.gamepad2.y=true; op.loop(); near(motors.get("flywheelA").velocity,0);
        op.gamepad2.left_bumper=op.gamepad2.dpad_up=true; op.loop();
        near(motors.get("flywheelA").velocity,50); near(motors.get("flywheelB").velocity,50);
        op.gamepad2.left_bumper=false; op.loop(); op.gamepad2.left_bumper=true; op.loop();
        near(motors.get("flywheelA").velocity,0); near(motors.get("flywheelB").velocity,0);
        check("UNCALIBRATED".equals(field(op,"autoSpeedStatus")),"Uncalibrated AUTO reports why it is off");
        Shooter shooter=(Shooter)field(op,"shooter"); near(shooter.getVelocityTolerance(),75);
        shooter.setStableTimeSeconds(0); shooter.setTargetVelocity(1100);
        motors.get("flywheelA").velocity=1070; motors.get("flywheelB").velocity=1130;
        check(shooter.isAtSpeed(),"Both motors within practical tolerance");
        shooter.setStableTimeSeconds(100); shooter.setTargetVelocity(1150);
        check(!shooter.isAtSpeed(),"Changed target restarts stability requirement");
        op.stop(); check(vision.stopped,"STOP ends camera polling");
        for(Motor motor:motors.values()){near(motor.power,0);near(motor.velocity,0);}
    }

    public static void main(String[] args) throws Exception {
        visionChecks(new File(args[0]));
        File directory = new File(args[0], "app-files"); check(directory.mkdir(), "Create app directory");
        File file = new File(directory, BallColorCalibration.FILE_NAME);
        Color color = new Color();
        BallColorSensor detector = new BallColorSensor(color.device, file);
        check(!detector.isCalibrated() && detector.getCalibrationStatus().startsWith("MISSING"), "Missing file");
        detector.update(); detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "No calibration = unknown");
        check(!detector.saveCalibration() && !file.exists(), "Incomplete calibration not saved");
        byte[] legacy = legacyCalibration();
        Files.write(file.toPath(), legacy);
        detector = new BallColorSensor(color.device, file);
        check(detector.isCalibrated() && detector.getCalibrationStatus().startsWith("LOADED"), "Automatic load");
        check(detector.getCalibration().getFormatVersion() == 1 && !detector.isRedCalibrated()
                && Arrays.equals(legacy, Files.readAllBytes(file.toPath())), "V1 loads without rewriting or inventing red");
        detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "First reading unconfirmed");
        detector.update(); check(detector.getBallType() == BallColorSensor.BallType.POLLEN, "Yellow confirmed");
        check(Double.isFinite(detector.getReadTimeMs()) && Double.isFinite(detector.getConfirmationTimeMs()), "Timing telemetry");
        int reads = color.reads; detector.getRedFraction(); detector.getStatus(); detector.getBallType();
        check(color.reads == reads && color.reads == color.distanceReads, "Cached telemetry and one read per channel set");
        color.rgb(.1f, .2f, .7f); detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Changed candidate clears old ball");
        detector.update(); check(detector.getBallType() == BallColorSensor.BallType.NECTAR, "Blue confirmed");
        color.rgb(.2f, .4f, .4f); detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Ambiguous/unmatched");
        detector.configureCalibration(new BallColorCalibration(calibration().empty,
                reference(.4, .5, .1, 3), reference(.45, .45, .1, 4)));
        color.rgb(.425f, .475f, .1f); detector.update(); detector.update();
        check(detector.getBestColorError() < BallColorCalibration.MAX_COLOR_DISTANCE
                && detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Margin rejects close equally likely colors");
        detector.configureCalibration(calibration());
        color.rgb(1, 0, 0); detector.update(); detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Uncalibrated red");
        color.rgb(.3f, .6f, .1f); color.distance = 15; detector.update(); detector.update();
        check(detector.getStatus().equals("NO_OBJECT_NEAR"), "Proximity gate");
        color.distance = Double.NaN; detector.update(); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.POLLEN, "Unreliable distance color-only fallback");
        color.failed = true; detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN && !detector.hasColorSample(), "Disconnect clears stale values");
        color.failed = false; detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Recovery confirms afresh");
        detector.update(); check(detector.getBallType() == BallColorSensor.BallType.POLLEN, "Recovery");
        color.rgb(Float.NaN, 0, 0); detector.update(); check(!detector.hasColorSample(), "Nonfinite rejected");
        color.rgb(0, 0, 0); detector.update(); check(detector.hasColorSample() && !detector.hasValidReading(), "Dark EMPTY remains capturable");
        check(new BallColorSensor(null, file).getBallType() == BallColorSensor.BallType.UNKNOWN, "Missing hardware safe");

        withRed().save(file);
        detector = new BallColorSensor(color.device, file);
        check(detector.isRedCalibrated() && detector.getCalibration().getFormatVersion() == 2, "V2 red round trip");
        sameReference(withRed().redNectar, detector.getCalibration().redNectar);
        color.rgb(.75f, .15f, .10f); color.distance = 4;
        detector.update(); check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Red requires confirmation");
        detector.update(); check(detector.getBallType() == BallColorSensor.BallType.NECTAR, "Red confirmed as NECTAR");
        color.rgb(.1f, .2f, .7f); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.NECTAR, "Blue and red share the same confirmed ball type");
        color.rgb(.3f, .6f, .1f); detector.update(); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.POLLEN, "Yellow preserved with red configured");
        color.rgb(.75f, .15f, .10f); color.distance = 15; detector.update(); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN && detector.getStatus().equals("NO_OBJECT_NEAR"), "Red proximity gate");
        detector.configureCalibration(new BallColorCalibration(calibration().empty, calibration().pollen,
                calibration().nectar, reference(.75, .15, .10, Double.NaN)));
        color.rgb(.3f, .6f, .1f); detector.update(); detector.update();
        check(detector.getStatus().equals("NO_OBJECT_NEAR"), "Missing red proximity must not disable yellow/blue gating");
        color.rgb(.75f, .15f, .10f); color.distance = Double.NaN; detector.update(); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.NECTAR, "Red color-only fallback when distance is unreliable");
        detector.configureCalibration(new BallColorCalibration(calibration().empty, calibration().pollen,
                calibration().nectar, reference(.35, .55, .1, 4)));
        color.rgb(.325f, .575f, .1f); color.distance = 4; detector.update(); detector.update();
        check(detector.getBestColorError() < BallColorCalibration.MAX_COLOR_DISTANCE
                && detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Red/yellow ambiguity rejected");
        detector.configureCalibration(new BallColorCalibration(reference(.2, .3, .5, 12), calibration().pollen,
                calibration().nectar, withRed().redNectar));
        color.rgb(.2f, .3f, .5f); detector.update(); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN, "Illuminated EMPTY rejected even within proximity gate");
        color.rgb(0, 0, 0); detector.update();
        check(detector.getBallType() == BallColorSensor.BallType.UNKNOWN && detector.getStatus().equals("WEAK_SIGNAL"), "Dark EMPTY never a ball");

        BallColorCalibration.Samples dark = new BallColorCalibration.Samples();
        for (int i = 0; i < 12; i++) dark.add(0, 0, 0, Double.NaN);
        BallColorCalibration darkCalibration = new BallColorCalibration(dark.average(), calibration().pollen, calibration().nectar);
        check(Double.isNaN(darkCalibration.getGateDistanceCm()), "No invented distance");
        dark.add(.2, .3, .5, 10); // Mixed dark and lit EMPTY is valid even with fraction sum < 1.
        new BallColorCalibration(dark.average(), calibration().pollen, calibration().nectar).save(file);
        near(BallColorCalibration.load(file).empty.red, .2 / 13);
        expectFailure(() -> new BallColorCalibration.Samples().average());
        expectFailure(() -> new BallColorCalibration(reference(0, 0, 0, 12), reference(0, 0, 0, 3), calibration().nectar));
        expectFailure(() -> new BallColorCalibration(calibration().empty, calibration().pollen, calibration().pollen));
        expectFailure(() -> new BallColorCalibration(reference(Double.NaN, 0, 0, 12), calibration().pollen, calibration().nectar));
        expectFailure(() -> new BallColorCalibration(calibration().empty, calibration().pollen, calibration().nectar,
                new BallColorCalibration.Reference(.75, .15, .1, 4, 11, 11)));
        expectFailure(() -> new BallColorCalibration(calibration().empty, calibration().pollen, calibration().nectar,
                calibration().pollen));
        withRed().save(file);
        byte[] redSaved = Files.readAllBytes(file.toPath());
        byte[] badRed = redSaved.clone(); java.nio.ByteBuffer.wrap(badRed).putDouble(129, Double.NaN);
        writeChecksummed(file, badRed);
        check(!new BallColorSensor(color.device, file).isCalibrated(), "Corrupt red values reject file, not silently dropped");
        byte[] badPresence = redSaved.clone(); badPresence[128] = 2; writeChecksummed(file, badPresence);
        check(!new BallColorSensor(color.device, file).isCalibrated(), "Invalid optional-red presence rejected");
        calibration().save(file);
        byte[] saved = Files.readAllBytes(file.toPath());
        check(BallColorCalibration.load(file).redNectar == null && BallColorCalibration.load(file).getFormatVersion() == 2,
                "V2 without red retains yellow/blue calibration");
        byte[] corrupt = saved.clone(); corrupt[12] ^= 1; Files.write(file.toPath(), corrupt);
        check(!new BallColorSensor(color.device, file).isCalibrated(), "Checksum corruption rejected");
        Files.write(file.toPath(), Arrays.copyOf(saved, 20));
        check(!new BallColorSensor(color.device, file).isCalibrated(), "Truncated file rejected");
        byte[] wrongVersion = saved.clone(); wrongVersion[7] = 99;
        CRC32 crc = new CRC32(); crc.update(wrongVersion, 0, wrongVersion.length - 8);
        try (DataOutputStream out = new DataOutputStream(new FileOutputStream(file))) {
            out.write(wrongVersion, 0, wrongVersion.length - 8); out.writeLong(crc.getValue());
        }
        check(!new BallColorSensor(color.device, file).isCalibrated(), "Version rejected despite valid checksum");
        byte[] invalidValues = saved.clone(); java.nio.ByteBuffer.wrap(invalidValues).putDouble(48, Double.NaN);
        crc.reset(); crc.update(invalidValues, 0, invalidValues.length - 8);
        try (DataOutputStream out = new DataOutputStream(new FileOutputStream(file))) {
            out.write(invalidValues, 0, invalidValues.length - 8); out.writeLong(crc.getValue());
        }
        check(!new BallColorSensor(color.device, file).isCalibrated(), "Nonfinite reference rejected despite valid checksum");
        calibration().save(file);
        BallColorSensor incomplete = new BallColorSensor(color.device, file); incomplete.clearCalibration();
        check(!incomplete.saveCalibration() && Arrays.equals(saved, Files.readAllBytes(file.toPath())), "Failed incomplete save preserves old calibration");
        File unwritableTarget = new File(directory, "directory-not-file"); check(unwritableTarget.mkdir(), "Create failure fixture");
        boolean failedSave = false;
        try { calibration().save(unwritableTarget); } catch (IOException expected) { failedSave = true; }
        check(failedSave && Arrays.equals(saved, Files.readAllBytes(file.toPath())), "Failed replacement leaves valid saved file");

        withRed().save(file);
        color.rgb(.3f, .6f, .1f); color.distance = 3;
        Adjustment adjustment = new Adjustment(); Map<String, Motor> motors = new HashMap<>();
        Teleop op = new Teleop(); initialize(op, hardware(directory, color, adjustment, motors));
        check(adjustment.calls == 0, "No init servo movement");
        reads = color.reads; op.loop(); check(adjustment.calls == 0 && color.reads == reads + 1, "One coordinator update");
        op.loop(); near(adjustment.position, .10);
        Shooter shooter = (Shooter) field(op, "shooter");
        long commandTime = (long) field(shooter, "adjustmentCommandNanos");
        op.loop(); check(adjustment.calls == 1 && (long) field(shooter, "adjustmentCommandNanos") == commandTime, "No repeated command/timer restart");
        shooter.setAdjustmentSettlingSeconds(3600); // A long test interval must not delay physical commands.
        color.rgb(.1f, .2f, .7f); op.loop(); near(adjustment.position, .10); op.loop(); near(adjustment.position, .00);
        check(!shooter.isAdjustmentSettled(), "Settling independent of servo command");
        int nectarCommands = adjustment.calls;
        commandTime = (long) field(shooter, "adjustmentCommandNanos");
        color.rgb(.75f, .15f, .10f); op.loop(); op.loop(); near(adjustment.position, .00);
        check(shooter.getSelectedBallType() == Shooter.BallType.NECTAR && adjustment.calls == nectarCommands
                && (long) field(shooter, "adjustmentCommandNanos") == commandTime, "Red selects same nectar preset without re-command or timer restart");
        color.rgb(.3f, .6f, .1f); op.loop(); op.loop(); near(adjustment.position, .10);
        color.rgb(.75f, .15f, .10f); op.loop(); near(adjustment.position, .10); op.loop(); near(adjustment.position, .00);
        rb(op); near(adjustment.position, .10); check(shooter.getSelectionMode() == Shooter.SelectionMode.MANUAL_POLLEN, "Manual pollen ignores red");
        op.gamepad2.right_bumper = true; op.loop(); int commanded = adjustment.calls; op.loop();
        check(adjustment.calls == commanded && shooter.getSelectionMode() == Shooter.SelectionMode.MANUAL_NECTAR, "RB hold cycles once");
        op.gamepad2.right_bumper = false; op.loop(); color.failed = true; rb(op);
        check(shooter.getSelectedBallType() == Shooter.BallType.UNKNOWN && !shooter.isReadyToFeed(), "AUTO disconnected cannot retain manual identification");
        color.failed = false;
        op.gamepad2.x = true; op.gamepad2.left_trigger = .3f; op.loop();
        near(motors.get("intake").power, .6); near(motors.get("transfer").power, .15);
        op.gamepad2.right_trigger = .3f; op.loop();
        near(motors.get("intake").power, -.50); near(motors.get("transfer").power, -.50);
        op.gamepad2.x = false; op.gamepad2.left_trigger = op.gamepad2.right_trigger = 0;
        op.gamepad2.y = op.gamepad2.left_bumper = op.gamepad2.dpad_up = true; op.loop();
        near(motors.get("flywheelA").velocity, 50); near(motors.get("flywheelB").velocity, 50);
        op.loop(); near(motors.get("flywheelA").velocity, 50);
        check(motors.get("flywheelA").direction == DcMotorSimple.Direction.FORWARD
                && motors.get("flywheelB").direction == DcMotorSimple.Direction.REVERSE, "Flywheel directions preserved");
        op.stop(); for (Motor motor : motors.values()) { near(motor.power, 0); near(motor.velocity, 0); }

        check(file.delete(), "Remove calibration fixture");
        adjustment = new Adjustment(); Teleop manual = new Teleop();
        initialize(manual, hardware(directory, null, adjustment, new HashMap<>()));
        rb(manual); near(adjustment.position, .10); rb(manual); near(adjustment.position, .00);
        rb(manual); check(((Shooter) field(manual, "shooter")).getSelectedBallType() == Shooter.BallType.UNKNOWN, "Manual without sensor/calibration; return AUTO unknown");

        FastColorTest test = new FastColorTest(); adjustment = new Adjustment();
        initialize(test, hardware(directory, color, adjustment, new HashMap<>()));
        test.gamepad1.dpad_up = true; test.loop(); check(!file.exists(), "Test cannot save incomplete groups"); test.gamepad1.dpad_up = false;
        color.rgb(0, 0, 0); color.distance = 12; test.gamepad1.x = true;
        for (int i = 0; i < 12; i++) test.loop();
        test.gamepad1.x = false; test.gamepad1.y = true; color.rgb(.3f, .6f, .1f); color.distance = 3;
        for (int i = 0; i < 12; i++) test.loop();
        test.gamepad1.y = false; test.gamepad1.b = true; color.rgb(.1f, .2f, .7f); color.distance = 4;
        for (int i = 0; i < 12; i++) test.loop();
        check(adjustment.calls == 0 && !file.exists(), "Collecting never moves/saves automatically");
        test.gamepad1.b = false; test.gamepad1.dpad_up = true; test.loop();
        check(file.exists() && BallColorCalibration.load(file).empty.samples == 12, "Explicit save includes dark EMPTY");
        near(BallColorCalibration.load(file).empty.red, 0);
        test.gamepad1.a = true; test.loop(); near(adjustment.position, .02);
        check(new BallColorSensor(color.device, file).isCalibrated(), "Saved test data reloads for competition");

        // Upgrade only red using a real v1 fixture, preserving every existing reference exactly.
        Files.write(file.toPath(), legacy);
        test = new FastColorTest(); adjustment = new Adjustment();
        initialize(test, hardware(directory, color, adjustment, new HashMap<>()));
        check(adjustment.calls == 0 && Arrays.equals(legacy, Files.readAllBytes(file.toPath())), "Loading v1 never moves or rewrites");
        test.gamepad1.left_bumper = test.gamepad1.y = test.gamepad1.a = true;
        test.loop();
        check(((BallColorCalibration.Samples) field(test, "redNectar")).getCount() == 0
                && ((BallColorCalibration.Samples) field(test, "pollen")).getCount() == 0
                && adjustment.calls == 0, "Conflicting sample buttons ignored; A cannot move while capturing");
        test.gamepad1.y = test.gamepad1.a = false;
        color.rgb(0, 0, 0); test.loop(); color.rgb(Float.NaN, 0, 0); test.loop();
        check(((BallColorCalibration.Samples) field(test, "redNectar")).getCount() == 0, "Weak and invalid red samples excluded");
        color.rgb(.75f, .15f, .10f); color.distance = 4;
        for (int i = 0; i < 11; i++) test.loop();
        test.gamepad1.left_bumper = false; test.gamepad1.dpad_up = true; test.loop();
        check(Arrays.equals(legacy, Files.readAllBytes(file.toPath())), "11 red samples cannot replace working calibration");
        test.gamepad1.dpad_up = false; test.loop();
        test.gamepad1.left_bumper = test.gamepad1.dpad_up = true; test.loop(); // Twelfth sample, but saving while capturing is blocked.
        check(Arrays.equals(legacy, Files.readAllBytes(file.toPath())), "Save while capturing blocked");
        test.gamepad1.left_bumper = false; test.loop();
        check(Arrays.equals(legacy, Files.readAllBytes(file.toPath())), "Held save does not trigger after capture release");
        test.gamepad1.dpad_up = false; test.loop(); test.gamepad1.dpad_up = true; test.loop();
        BallColorCalibration upgraded = BallColorCalibration.load(file);
        check(upgraded.getFormatVersion() == 2 && upgraded.redNectar.samples == 12, "Explicit red-only save upgrades v1 with 12 valid samples");
        sameReference(calibration().empty, upgraded.empty); sameReference(calibration().pollen, upgraded.pollen);
        sameReference(calibration().nectar, upgraded.nectar);
        test.gamepad1.a = true; test.loop(); near(adjustment.position, .02);
        color.rgb(.3f, .6f, .1f); test.loop(); test.loop(); near(adjustment.position, .30);
        color.rgb(.1f, .2f, .7f); test.loop(); test.loop(); near(adjustment.position, .02);
        check(new BallColorSensor(color.device, file).isRedCalibrated(), "Saved red reloads for competition");
        dualFlowerChecks(directory);
        servoCalibrationChecks(directory);
        competitionControlChecks(directory);
        System.out.println("PASS: " + checks + " focused checks; no physical hardware or follower constructed.");
    }
}
