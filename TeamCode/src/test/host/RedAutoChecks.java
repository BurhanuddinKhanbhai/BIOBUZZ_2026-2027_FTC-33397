package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.sensors.BallColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.lang.reflect.*;
import java.util.*;
import java.util.function.LongSupplier;

/** Synthetic host checks. None of the numbers below are robot calibration or field measurements. */
public final class RedAutoChecks {
    private static int checks;
    private static void check(boolean condition, String reason) { checks++; if (!condition) throw new AssertionError(reason); }
    private static void near(double actual, double expected) { check(Math.abs(actual - expected) < 1e-6, actual + " != " + expected); }
    private static Object field(Object object, String name) throws Exception {
        Field f = object.getClass().getDeclaredField(name); f.setAccessible(true); return f.get(object);
    }
    private static void reject(Runnable action, String message) {
        boolean rejected = false;
        try { action.run(); } catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, message);
    }
    @SuppressWarnings("unchecked") private static <T> T proxy(Class<T> type, InvocationHandler handler) {
        return (T) Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, handler);
    }
    private static final class Clock implements LongSupplier {
        long now = 1_000_000_000L;
        public long getAsLong() { return now; }
        void advance(double seconds) { now += (long) (seconds * 1_000_000_000L); }
    }
    private static final class Motor {
        final DcMotorEx device;
        double power, velocity; int zeroAttempts, nonzeroWrites;
        boolean throwZero; DcMotor.RunMode mode; DcMotorSimple.Direction direction;
        Motor(DcMotorController controller, int port) {
            device = proxy(DcMotorEx.class, (p, m, a) -> {
                switch (m.getName()) {
                    case "getController": return controller;
                    case "getPortNumber": return port;
                    case "setPower":
                        double next = (double) a[0];
                        if (next == 0) { zeroAttempts++; if (throwZero) throw new IllegalStateException("motor zero failure"); }
                        else nonzeroWrites++;
                        power = next; return null;
                    case "getPower": return power;
                    case "setVelocity": velocity = (double) a[0]; if (velocity != 0) nonzeroWrites++; return null;
                    case "getVelocity": return velocity;
                    case "setMode": mode = (DcMotor.RunMode) a[0]; check(mode != DcMotor.RunMode.STOP_AND_RESET_ENCODER, "No encoder reset"); return null;
                    case "getMode": return mode;
                    case "setDirection": direction = (DcMotorSimple.Direction) a[0]; return null;
                    case "getDirection": return direction;
                    case "setZeroPowerBehavior": return null;
                    default: throw new AssertionError("Unexpected motor operation: " + m.getName());
                }
            });
        }
    }
    private static final class Adjustment {
        final Servo device; int calls; double position = Double.NaN;
        Adjustment(ServoController controller, int port) {
            device = proxy(Servo.class, (p, m, a) -> {
                switch (m.getName()) {
                    case "getController": return controller;
                    case "getPortNumber": return port;
                    case "setDirection": return null;
                    case "setPosition": calls++; position = (double) a[0]; return null;
                    default: throw new AssertionError("Unexpected servo operation: " + m.getName());
                }
            });
        }
    }
    private static RedAuto.Settings configured() {
        RedAuto.Settings s = new RedAuto.Settings();
        s.routeAndMechanismsVerified = true;
        s.startX = 300; s.startY = 100; s.fieldWidth = 400; s.fieldHeight = 200;
        s.sweptRobotRadius = 10; s.boundaryMargin = 2; s.intake2Distance = 24;
        s.flower2Lowered = .7; s.flower2Direction = Servo.Direction.REVERSE; s.flowerSettlingSeconds = .1;
        s.shoot3.attempts = 2;
        for (RedAuto.FeedSettings f : new RedAuto.FeedSettings[]{s.shoot1, s.shoot2, s.shoot3}) {
            f.onSeconds = .1; f.gapSeconds = .05; f.timeoutSeconds = 8;
        }
        return s;
    }
    private static final class Fixture {
        final Clock clock = new Clock(); final RedAuto.Settings settings; final RedAuto auto;
        final HardwareMap map = new HardwareMap(); final Map<String, Motor> motors = new LinkedHashMap<>();
        final Adjustment flower1, flower2, adjust;
        Fixture(RedAuto.Settings s) {
            settings = s;
            DcMotorController left = proxy(DcMotorController.class, (p,m,a) -> null);
            DcMotorController right = proxy(DcMotorController.class, (p,m,a) -> null);
            ServoController leftServo = proxy(ServoController.class, (p,m,a) -> null);
            ServoController rightServo = proxy(ServoController.class, (p,m,a) -> null);
            add("frontLeft", left, 0); add("backLeft", left, 1); add("flywheelA", left, 2); add("transfer", left, 3);
            add("frontRight", right, 0); add("backRight", right, 1); add("intake", right, 2); add("flywheelB", right, 3);
            flower1 = new Adjustment(rightServo, 0); flower2 = new Adjustment(leftServo, 0); adjust = new Adjustment(leftServo, 1);
            map.put("flowerServo", flower1.device); map.put("flowerServo2", flower2.device); map.put("shooterAdjust", adjust.device);
            map.put("ballColor", new Object());
            auto = new RedAuto(s, clock); auto.hardwareMap = map;
            auto.telemetry = proxy(Telemetry.class, (p,m,a) -> m.getReturnType() == boolean.class ? false : null);
            Constants.issue = null; Follower.instance = null; Shooter.instance = null;
        }
        void add(String name, DcMotorController c, int port) { Motor motor = new Motor(c, port); motors.put(name, motor); map.put(name, motor.device); }
        void init() { auto.init(); }
        void start() { init(); auto.start(); }
        Follower follower() { return Follower.instance; }
        RedAutoPlan.Step step() throws Exception { return (RedAutoPlan.Step) field(auto, "step"); }
        boolean running() throws Exception { return (boolean) field(auto, "running"); }
        String error() throws Exception { return (String) field(auto, "error"); }
        void tick() { clock.advance(.05); auto.loop(); }
        void completeMotion() throws Exception {
            follower().pose = (Pose) field(auto, "target"); follower().busy = false;
            if (field(auto, "activePath") != null) ((Path) field(auto, "activePath")).t = 1;
            follower().speed = follower().angular = 0;
        }
        void approach(RedAutoPlan.Step target) throws Exception {
            for (int i = 0; i < 500 && step() != target && running(); i++) {
                if (RedAutoPlan.isDrive(step()) || RedAutoPlan.isTurn(step())) completeMotion();
                tick();
            }
            check(step() == target && running(), "Reach fixture step " + target + ": " + error());
        }
        void allZero() {
            for (Motor m : motors.values()) near(m.power, 0);
            if (Shooter.instance != null && !Shooter.instance.throwStop) {
                near(motors.get("flywheelA").velocity, 0); near(motors.get("flywheelB").velocity, 0);
            }
        }
        void failed(String text) throws Exception {
            check(!running() && error().contains(text), "Expected failure " + text + ", got " + error());
            allZero();
            int updates = follower() == null ? 0 : follower().updates;
            tick(); auto.start(); tick();
            check(follower() == null || follower().updates == updates, "Failure cannot resume follower");
        }
    }
    private static void geometry() {
        RedAutoPlan p = new RedAutoPlan(300,100,24,400,200,10,2);
        double[][] expected = {{300,100,Math.PI},{300,100,1.5*Math.PI},{300,52,1.5*Math.PI},
                {300,124,1.5*Math.PI},{300,124,Math.PI},{180,124,Math.PI},{180,124,0},{180,124,0},
                {180,124,0},{180,124,Math.PI},{192,124,Math.PI},{192,112,Math.PI},{168,112,Math.PI},
                {180,112,Math.PI},{180,112,0},{180,112,0}};
        check(RedAutoPlan.Step.values().length == 16, "All 16 states");
        for (RedAutoPlan.Step s : RedAutoPlan.Step.values()) {
            Pose target = p.getTarget(s); double[] e = expected[s.ordinal()];
            near(target.getX(), e[0]); near(target.getY(), e[1]); near(target.getHeading(), e[2]); check(p.contains(target), "Contained " + s);
        }
        near(RedAutoPlan.turnRadians(RedAutoPlan.Step.TURN_LEFT_90), Math.PI/2);
        near(RedAutoPlan.turnRadians(RedAutoPlan.Step.TURN_RIGHT_90), -Math.PI/2);
        for (RedAutoPlan.Step s : new RedAutoPlan.Step[]{RedAutoPlan.Step.TURN_180_1,RedAutoPlan.Step.TURN_180_2,RedAutoPlan.Step.TURN_180_3}) near(RedAutoPlan.turnRadians(s),Math.PI);
        reject(() -> new RedAutoPlan(135,70,12,144,144,12,0), "Wall-flush turn clearance rejected");
        reject(() -> new RedAutoPlan(100,100,12,144,144,10,2), "120-inch path outside field rejected");
        reject(() -> new RedAutoPlan(300,20,24,400,200,10,2), "First collection endpoint outside field rejected");
        reject(() -> new RedAutoPlan(300,100,300,400,200,10,2), "Late collection outside field rejected");
        reject(() -> new RedAutoPlan(Double.NaN,100,24,400,200,10,2), "NaN start rejected");
        reject(() -> new RedAutoPlan(300,100,0,400,200,10,2), "Zero collection distance rejected");
        reject(() -> new RedAutoPlan(300,100,24,Double.POSITIVE_INFINITY,200,10,2), "Infinite dimensions rejected");
        check(!p.contains(new Pose(Double.NaN, 100, 0)) && !p.contains(null), "Invalid feedback outside containment");
    }
    private static void fullSequence() throws Exception {
        Fixture f = new Fixture(configured()); f.init();
        check(f.flower1.calls == 0 && f.flower2.calls == 0 && f.adjust.calls == 0, "No servo movement in init");
        near(f.motors.get("frontLeft").power, 0); f.auto.start();
        near(f.motors.get("flywheelA").velocity, 1500); near(f.motors.get("flywheelB").velocity, 1500);
        check(f.motors.get("frontLeft").direction == DcMotorSimple.Direction.FORWARD
                && f.motors.get("frontRight").direction == DcMotorSimple.Direction.REVERSE, "Executable drive directions preserved");
        check(f.motors.get("transfer").mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "Shared transfer channel mode retained");
        List<RedAutoPlan.Step> entered = new ArrayList<>();
        for (int i = 0; i < 500 && f.running(); i++) {
            RedAutoPlan.Step s = f.step();
            if (entered.isEmpty() || entered.get(entered.size()-1) != s) entered.add(s);
            boolean collecting = s == RedAutoPlan.Step.INTAKE_1 || s == RedAutoPlan.Step.INTAKE_2;
            near(f.motors.get("intake").power, collecting ? .35 : 0);
            check(f.motors.get("transfer").power == 0 || RedAutoPlan.isShoot(s), "Transfer only during shooting");
            if (s.ordinal() < RedAutoPlan.Step.FLOWER_DOWN.ordinal()) check(f.flower1.calls == 0 && f.flower2.calls == 0, "Flower waits until step 9");
            if (RedAutoPlan.isTurn(s)) near(f.follower().turnDelta, RedAutoPlan.turnRadians(s));
            if (RedAutoPlan.isDrive(s)) near(((Path) field(f.auto, "activePath")).heading, ((Pose) field(f.auto, "target")).getHeading());
            if (RedAutoPlan.isDrive(s) || RedAutoPlan.isTurn(s)) f.completeMotion();
            int updates = f.follower().updates, reads = BallColorSensor.instance.reads;
            f.tick();
            check(f.follower().updates == updates + 1 && BallColorSensor.instance.reads == reads + 1, "One follower and color update per active loop");
        }
        check(entered.equals(Arrays.asList(RedAutoPlan.Step.values())), "Exact state order " + entered);
        check(f.auto.stopRequested && ((String) field(f.auto,"status")).startsWith("COMPLETE"), "Completed bounded sequence");
        check(f.follower().pathCalls == 7 && f.follower().turnCalls == 5, "Seven translations and five turns through Pedro");
        check(f.motors.get("transfer").nonzeroWrites == 10, "Exactly four, four and two requested feed pulses");
        check(f.flower1.calls == 1 && f.flower2.calls == 1, "Dual flower command issued once");
        near(f.flower1.position,.83); near(f.flower2.position,.7);
        check((int) field(f.auto,"attemptsCompleted") == 2, "Final count is configured feed attempts only");
        f.allZero(); int calls = f.follower().stopCalls; f.auto.stop(); f.tick(); check(f.follower().stopCalls == calls, "Shutdown latched once");
    }
    public static void main(String[] args) throws Exception {
        geometry(); fullSequence();
        Fixture f = new Fixture(new RedAuto.Settings()); f.init(); f.auto.start();
        check(!f.running() && f.follower() == null && f.error().contains("flowerServo2"), "Unconfigured run blocked with reasons");
        check(f.motors.values().stream().allMatch(m -> m.nonzeroWrites == 0) && f.flower1.calls == 0 && f.flower2.calls == 0, "Blocked initialization commands no movement");
        f = new Fixture(configured()); f.map.remove("flowerServo2"); f.start();
        check(!f.running() && f.follower() == null && f.error().contains("Missing servo: flowerServo2"), "Missing hardware blocks complete sequence");
        f = new Fixture(configured()); Constants.issue = "review guard closed"; f.start();
        check(!f.running() && f.follower() == null && f.error().contains("review guard closed"), "Closed production-like guard respected by controller");
        f = new Fixture(configured()); f.init(); f.auto.gamepad1.b = true; f.auto.start(); f.failed("emergency");
        check(f.motors.values().stream().allMatch(m -> m.nonzeroWrites == 0), "Held abort prevents startup spin");
        f = new Fixture(configured()); f.init(); f.auto.stop(); f.auto.start(); f.tick();
        check(!f.running() && f.motors.values().stream().allMatch(m -> m.nonzeroWrites == 0), "Stop before start cannot resurrect motors");

        f = new Fixture(configured()); f.start(); Shooter.instance.ready = false;
        for (int i=0;i<4;i++) f.tick(); near(f.motors.get("transfer").power,0);
        Shooter.instance.ready = true; BallColorSensor.instance.type = BallColorSensor.BallType.UNKNOWN;
        for (int i=0;i<4;i++) f.tick(); near(f.motors.get("transfer").power,0);
        BallColorSensor.instance.type = BallColorSensor.BallType.POLLEN; Shooter.instance.settled = false; f.tick(); near(f.motors.get("transfer").power,0);
        Shooter.instance.settled = true; f.tick(); near(f.motors.get("transfer").power,.35);
        check((int)field(f.auto,"attemptsCompleted")==0,"Started pulse is not completed attempt");
        Shooter.instance.ready = false; f.tick(); f.failed("Readiness lost");
        check((int)field(f.auto,"attemptsCompleted")==0,"Failed partial pulse not counted");

        f = new Fixture(configured()); f.start(); f.tick(); BallColorSensor.instance.type = BallColorSensor.BallType.NECTAR;
        int servoCalls=f.adjust.calls; f.tick(); f.failed("Different ball"); check(f.adjust.calls==servoCalls,"Abort before changing adjustment during feed");
        f = new Fixture(configured()); f.start(); BallColorSensor.instance.sample=false; f.tick(); f.failed("Color sensor");
        f = new Fixture(configured()); f.start(); f.auto.gamepad2.b=true; int updates=f.follower().updates; f.tick(); f.failed("emergency");
        check(f.follower().updates==updates,"Abort checked before follower output");
        f = new Fixture(configured()); f.start(); f.clock.advance(30); f.auto.loop(); f.failed("deadline");
        f = new Fixture(configured()); f.start(); f.clock.advance(.3); f.auto.loop(); f.failed("Control-loop gap");
        f = new Fixture(configured()); f.start(); f.follower().localizer.valid=false; f.tick(); f.failed("invalid localizer");
        f = new Fixture(configured()); f.start(); f.follower().pose=new Pose(-1,100,Math.PI); f.tick(); f.failed("field/robot clearance");
        f = new Fixture(configured()); f.start(); f.follower().speed=Double.NaN; f.tick(); f.failed("Invalid localization/velocity");
        f = new Fixture(configured()); f.start(); f.motors.get("flywheelB").velocity=Double.NaN; f.tick(); f.failed("Invalid flywheel");

        f = new Fixture(configured()); f.start(); f.approach(RedAutoPlan.Step.TURN_LEFT_90);
        f.follower().busy=false; f.tick(); f.failed("before verified destination");
        f = new Fixture(configured()); f.start(); f.approach(RedAutoPlan.Step.INTAKE_1);
        f.completeMotion(); ((Path)field(f.auto,"activePath")).t=.5; f.tick(); f.failed("before verified destination");
        f = new Fixture(configured()); f.start(); f.approach(RedAutoPlan.Step.TURN_LEFT_90);
        for (int i=0;i<45 && f.running();i++) f.tick(); f.failed("No localization progress");
        RedAuto.Settings timed=configured();timed.turnTimeout=.1;
        f = new Fixture(timed); f.start(); f.approach(RedAutoPlan.Step.TURN_LEFT_90);
        updates=f.follower().updates; f.clock.advance(.15); f.auto.loop(); f.failed("Action timeout");
        check(f.follower().updates==updates,"Timeout prevents additional follower update");
        RedAuto.Settings invalid=configured();invalid.endpointTimeoutMs=invalid.pathTimeout*1000;
        f = new Fixture(invalid);f.start();check(!f.running() && f.error().contains("endpoint timeout"),"Internal timeout cannot precede watchdog");

        f = new Fixture(configured()); f.start(); f.approach(RedAutoPlan.Step.TURN_LEFT_90);
        f.completeMotion(); f.follower().angular=.5; f.tick();
        check(f.running() && f.step()==RedAutoPlan.Step.TURN_LEFT_90,"Turn waits for actual settling");
        int stops=f.follower().stopCalls, turns=f.follower().turnCalls;
        f.tick(); f.tick(); check(f.follower().stopCalls==stops && f.follower().turnCalls==turns,"Settling never restarts or repeatedly stops turn");
        f.follower().angular=0; f.tick();check(f.step()==RedAutoPlan.Step.INTAKE_1,"Settled turn advances once");

        f = new Fixture(configured()); f.start(); f.approach(RedAutoPlan.Step.INTAKE_1); f.tick();
        f.follower().throwStop=true; Shooter.instance.throwStop=true; f.motors.get("frontLeft").throwZero=true;
        Map<String,Integer> before=new HashMap<>();for(Map.Entry<String,Motor> e:f.motors.entrySet())before.put(e.getKey(),e.getValue().zeroAttempts);
        int flowerCommands=f.flower1.calls+f.flower2.calls;f.auto.stop();
        for(Map.Entry<String,Motor> e:f.motors.entrySet())check(e.getValue().zeroAttempts>before.get(e.getKey()),"Independent zero attempt for "+e.getKey());
        check(f.flower1.calls+f.flower2.calls==flowerCommands,"No shutdown servo motion");
        check(f.error().contains("stop failed"),"Shutdown errors reported");
        updates=f.follower().updates;f.tick();f.auto.start();f.tick();check(f.follower().updates==updates,"Failed cancellation never resumes motion");
        System.out.println("PASS: "+checks+" autonomous checks. Real controller/route and Pedro geometry; mocked follower/readiness/hardware. No physical robot tested.");
    }
}
