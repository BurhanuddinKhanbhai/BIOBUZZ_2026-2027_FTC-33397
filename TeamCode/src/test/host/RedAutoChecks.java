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
            Follower.instance = null; Shooter.instance = null;
        }
        void add(String name, DcMotorController c, int port) { Motor motor = new Motor(c, port); motors.put(name, motor); map.put(name, motor.device); }
        void init() { auto.init(); }
        void start() { init(); auto.start(); }
        Follower follower() { return Follower.instance; }
        RedAutoPlan.Step step() throws Exception { return (RedAutoPlan.Step) field(auto, "step"); }
        boolean running() throws Exception { return (boolean) field(auto, "running"); }
        void tick() { clock.advance(.05); auto.loop(); }
        void completeMotion() throws Exception {
            follower().pose = (Pose) field(auto, "target"); follower().busy = false;
            if (field(auto, "activePath") != null) ((Path) field(auto, "activePath")).t = 1;
        }
        void approach(RedAutoPlan.Step target) throws Exception {
            for (int i = 0; i < 500 && step() != target && running(); i++) {
                if (RedAutoPlan.isDrive(step()) || RedAutoPlan.isTurn(step())) completeMotion();
                tick();
            }
            check(step() == target && running(), "Reach fixture step " + target);
        }
        void allZero() {
            for (Motor m : motors.values()) near(m.power, 0);
            if (Shooter.instance != null && !Shooter.instance.throwStop) {
                near(motors.get("flywheelA").velocity, 0); near(motors.get("flywheelB").velocity, 0);
            }
        }
    }
    private static void geometry() {
        RedAutoPlan p = new RedAutoPlan(48);
        double[][] expected = {{0,0,0},{0,0,Math.PI/2},{0,48,Math.PI/2},
                {0,-24,Math.PI/2},{0,-24,0},{120,-24,0},{120,-24,Math.PI},{120,-24,Math.PI},
                {120,-24,Math.PI},{120,-24,0},{108,-24,0},{108,-12,0},{156,-12,0},
                {144,-12,0},{144,-12,Math.PI},{144,-12,Math.PI}};
        check(RedAutoPlan.Step.values().length == 16, "All 16 states");
        for (RedAutoPlan.Step step : RedAutoPlan.Step.values()) {
            Pose t = p.getTarget(step); double[] e = expected[step.ordinal()];
            near(t.getX(),e[0]);near(t.getY(),e[1]);near(t.getHeading(),e[2]);
        }
        near(p.getStart().getX(),0);near(p.getStart().getY(),0);near(p.getStart().getHeading(),0);
        near(RedAutoPlan.turnRadians(RedAutoPlan.Step.TURN_RIGHT_90),-Math.PI/2);
    }
    private static void fullSequence() throws Exception {
        Fixture f = new Fixture(new RedAuto.Settings()); f.init();
        check(f.flower1.calls==0 && f.flower2.calls==0 && f.adjust.calls==0,"No INIT servo commands");
        check(f.motors.get("frontLeft").direction==DcMotorSimple.Direction.FORWARD
                && f.motors.get("frontRight").direction==DcMotorSimple.Direction.REVERSE,"Drive directions unchanged");
        check(f.motors.get("intake").mode==DcMotor.RunMode.RUN_WITHOUT_ENCODER,"Shared intake channel mode unchanged");
        f.auto.start();
        List<RedAutoPlan.Step> entered=new ArrayList<>(); int bursts=0; long burstStart=0;
        for(int i=0;i<600 && f.running();i++) {
            RedAutoPlan.Step step=f.step();
            if(entered.isEmpty() || entered.get(entered.size()-1)!=step) {
                entered.add(step);
                if(RedAutoPlan.isShoot(step)) BallColorSensor.instance.type=step==RedAutoPlan.Step.SHOOT_2
                        ? BallColorSensor.BallType.NECTAR : BallColorSensor.BallType.POLLEN;
            }
            boolean before=(boolean)field(f.auto,"feeding");
            if(before) {near(f.motors.get("intake").power,1);near(f.motors.get("transfer").power,.90);}
            else if(step==RedAutoPlan.Step.INTAKE_1 || step==RedAutoPlan.Step.INTAKE_2) near(f.motors.get("intake").power,1);
            else near(f.motors.get("intake").power,0);
            if(!RedAutoPlan.isShoot(step)) near(f.motors.get("transfer").power,0);
            if(RedAutoPlan.isTurn(step)) near(f.follower().turnDelta,RedAutoPlan.turnRadians(step));
            if(RedAutoPlan.isDrive(step)) near(((Path)field(f.auto,"activePath")).heading,((Pose)field(f.auto,"target")).getHeading());
            if(RedAutoPlan.isDrive(step) || RedAutoPlan.isTurn(step)) f.completeMotion();
            int updates=f.follower().updates, reads=BallColorSensor.instance.reads;
            f.tick();
            check(f.follower().updates==updates+1 && BallColorSensor.instance.reads==reads+1,"One follower/sensor update per active loop");
            boolean after=(boolean)field(f.auto,"feeding");
            if(!before && after) {
                bursts++;burstStart=f.clock.now;
                double speed=step==RedAutoPlan.Step.SHOOT_2 ? 1200 : 1100;
                near(f.motors.get("flywheelA").velocity,speed);near(f.motors.get("flywheelB").velocity,speed);
                near(f.adjust.position,step==RedAutoPlan.Step.SHOOT_2 ? .00 : .10);
            }
            if(before && !after) near((f.clock.now-burstStart)/1e9,4.0);
        }
        check(entered.equals(Arrays.asList(RedAutoPlan.Step.values())),"All 16 steps in exact order");
        check(bursts==3 && f.motors.get("transfer").nonzeroWrites==3,"Exactly three continuous feed bursts");
        check(f.follower().pathCalls==7 && f.follower().turnCalls==5,"Seven Pedro paths and five Pedro turns");
        check(f.auto.stopRequested && !f.running(),"Finishes and requests normal SDK stop");
        check(f.flower1.calls==1 && f.flower2.calls==1,"Flower command once per arm");
        near(f.flower1.position,.48);near(f.flower2.position,.48);f.allZero();
    }
    public static void main(String[] args) throws Exception {
        geometry();fullSequence();
        Fixture f=new Fixture(new RedAuto.Settings());f.start();
        Shooter.instance.ready=false;f.tick();near(f.motors.get("transfer").power,0);
        Shooter.instance.ready=true;BallColorSensor.instance.type=BallColorSensor.BallType.UNKNOWN;f.tick();near(f.motors.get("transfer").power,0);
        BallColorSensor.instance.type=BallColorSensor.BallType.POLLEN;Shooter.instance.settled=false;f.tick();near(f.motors.get("transfer").power,0);
        Shooter.instance.settled=true;f.tick();near(f.motors.get("transfer").power,.90);
        // Once started, retain the original first-ball preset/speed and continuous feed behavior.
        BallColorSensor.instance.type=BallColorSensor.BallType.NECTAR;Shooter.instance.ready=false;
        f.clock.advance(2);f.auto.loop();near(f.motors.get("transfer").power,.90);near(f.motors.get("flywheelA").velocity,1100);
        f.clock.advance(2);f.auto.loop();check(f.step()==RedAutoPlan.Step.TURN_LEFT_90,"Feed ends after four seconds despite loop gap");
        int turns=f.follower().turnCalls;
        f.clock.advance(40);f.auto.loop();
        check(f.running() && f.step()==RedAutoPlan.Step.TURN_LEFT_90 && f.follower().turnCalls==turns,
                "No custom gap, deadline, turn watchdog or no-progress interruption");
        f.follower().pose=new Pose(0,0,.1);f.tick();
        check(f.step()==RedAutoPlan.Step.TURN_LEFT_90,"Slight turn cannot advance while Pedro is busy");
        f.completeMotion();f.tick();check(f.step()==RedAutoPlan.Step.INTAKE_1,"Pedro completion advances normally");
        f.auto.gamepad2.b=true;int updates=f.follower().updates;f.tick();
        check(!f.running() && f.auto.stopRequested && f.follower().updates==updates,"Operator abort precedes further movement");f.allZero();
        f=new Fixture(new RedAuto.Settings());f.init();f.auto.gamepad1.b=true;f.auto.start();
        check(!f.running() && f.motors.values().stream().allMatch(m->m.nonzeroWrites==0),"Held B prevents start");
        f=new Fixture(new RedAuto.Settings());f.start();f.approach(RedAutoPlan.Step.FLOWER_DOWN);
        f.clock.advance(.95);f.auto.loop();check(f.step()==RedAutoPlan.Step.FLOWER_DOWN,"Flower duration retained");
        f.clock.advance(.05);f.auto.loop();check(f.step()==RedAutoPlan.Step.TURN_180_2,"Flower advances after one second");
        f=new Fixture(new RedAuto.Settings());f.start();f.approach(RedAutoPlan.Step.INTAKE_1);f.tick();
        f.follower().throwStop=true;f.motors.get("frontLeft").throwZero=true;
        int flowerCalls=f.flower1.calls+f.flower2.calls;f.auto.stop();
        for(String name:new String[]{"frontRight","backLeft","backRight","intake","transfer"}) near(f.motors.get(name).power,0);
        near(f.motors.get("flywheelA").velocity,0);near(f.motors.get("flywheelB").velocity,0);
        check(f.flower1.calls+f.flower2.calls==flowerCalls,"STOP never commands servos");
        check(!f.running(),"STOP ends sequence despite failed cancellation");
        System.out.println("PASS: "+checks+" autonomous checks; real controller/route, mocked follower/readiness/hardware. No robot operated.");
    }
}
