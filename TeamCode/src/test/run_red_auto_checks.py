#!/usr/bin/env python3
"""Deterministic host checks: real route/controller/geometry, mocked follower and sensor hardware.
No physical follower is constructed and no production calibration guard is changed.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import zipfile

project = Path(__file__).resolve().parents[3]
cache = Path.home() / '.gradle/caches/modules-2/files-2.1'
sdk_aar = next((cache / 'org.firstinspires.ftc/RobotCore/11.1.0').glob('*/*.aar'))
core = next(p for p in (cache / 'com.pedropathing/core/2.1.2').glob('*/*.jar')
            if not p.name.endswith(('-sources.jar', '-javadoc.jar')))
java_home = Path(os.environ.get('JAVA_HOME', '/Applications/Android Studio.app/Contents/jbr/Contents/Home'))
javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else shutil.which('javac')
java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else shutil.which('java')
stubs = {
'com/qualcomm/robotcore/hardware/HardwareMap.java': '''package com.qualcomm.robotcore.hardware;
public class HardwareMap { private final java.util.Map<String,Object> devices=new java.util.HashMap<>();
 public void put(String n,Object d){devices.put(n,d);} public void remove(String n){devices.remove(n);}
 public <T>T tryGet(Class<T> t,String n){Object d=devices.get(n);return t.isInstance(d)?t.cast(d):null;}
 public <T>T get(Class<T> t,String n){T d=tryGet(t,n);if(d==null)throw new IllegalArgumentException(n);return d;}}
''',
'com/qualcomm/robotcore/hardware/Gamepad.java': '''package com.qualcomm.robotcore.hardware;
public class Gamepad { public boolean b; }''',
'com/qualcomm/robotcore/eventloop/opmode/OpMode.java': '''package com.qualcomm.robotcore.eventloop.opmode;
public abstract class OpMode {
 public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
 public org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
 public com.qualcomm.robotcore.hardware.Gamepad gamepad1=new com.qualcomm.robotcore.hardware.Gamepad(),gamepad2=new com.qualcomm.robotcore.hardware.Gamepad();
 public boolean stopRequested; public void requestOpModeStop(){stopRequested=true;}
 public abstract void init(); public abstract void loop(); public void init_loop(){} public void start(){} public void stop(){}
}''',
'com/qualcomm/robotcore/util/RobotLog.java': '''package com.qualcomm.robotcore.util;
public class RobotLog { public static int errors; public static void ee(String t,Throwable e,String s){errors++;} }''',
'org/firstinspires/ftc/teamcode/sensors/BallColorSensor.java': '''package org.firstinspires.ftc.teamcode.sensors;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class BallColorSensor {
 public enum BallType {UNKNOWN,POLLEN,NECTAR}
 public static BallColorSensor instance; public boolean available=true,calibrated=true,red=true,sample=true;
 public BallType type=BallType.POLLEN; public int reads;
 public BallColorSensor(HardwareMap m){instance=this;available=m.tryGet(Object.class,"ballColor")!=null;}
 public void update(){reads++;} public boolean isAvailable(){return available;}
 public boolean isCalibrated(){return calibrated;} public boolean isRedCalibrated(){return red;}
 public boolean hasColorSample(){return sample&&available;} public BallType getBallType(){return type;}
 public String getStatus(){return hasColorSample()?"SAMPLE":"READ_ERROR";}
 public String getCalibrationStatus(){return calibrated?"CALIBRATED":"MISSING";}
}''',
'org/firstinspires/ftc/teamcode/subsystems/Shooter.java': '''package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.sensors.BallColorSensor.BallType;
public class Shooter {
 public enum SelectionMode {AUTO,MANUAL}
 public static Shooter instance; public boolean ready=true,settled=true,throwStop;
 public int stopCalls; public double target; private double pollen,nectar,last=Double.NaN;
 private final DcMotorEx a,b; private final Servo servo; private BallType selected=BallType.UNKNOWN;
 public SelectionMode mode=SelectionMode.AUTO;
 public Shooter(HardwareMap m,DcMotorSimple.Direction da,DcMotorSimple.Direction db,double tolerance,double stable,Servo.Direction sd,double settle){
  instance=this;a=m.get(DcMotorEx.class,"flywheelA");b=m.get(DcMotorEx.class,"flywheelB");servo=m.get(Servo.class,"shooterAdjust");
  a.setDirection(da);b.setDirection(db);a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);stop();}
 public void configureAdjustmentPositions(double p,double n){pollen=p;nectar=n;}
 public void setAutoMode(){mode=SelectionMode.AUTO;selected=BallType.UNKNOWN;}
 public void setSelectionMode(SelectionMode m){mode=m;}
 public void update(BallType t){if(mode==SelectionMode.AUTO){selected=t;if(t!=BallType.UNKNOWN){double p=t==BallType.POLLEN?pollen:nectar;if(p!=last){servo.setPosition(p);last=p;}}}}
 public boolean isAtSpeed(){return ready&&target>0;} public boolean isAdjustmentSettled(){return settled;}
 public boolean isReadyToFeed(){return isAtSpeed()&&settled&&selected!=BallType.UNKNOWN;}
 public void setTargetVelocity(double t){target=t;a.setVelocity(t);b.setVelocity(t);}
 public double getTargetVelocity(){return target;} public double getVelocityA(){return a.getVelocity();} public double getVelocityB(){return b.getVelocity();}
 public void stop(){stopCalls++;if(throwStop)throw new IllegalStateException("shooter stop");setTargetVelocity(0);}
}''',
'org/firstinspires/ftc/teamcode/pedroPathing/SafeThreeWheelLocalizer.java': '''package org.firstinspires.ftc.teamcode.pedroPathing;
public class SafeThreeWheelLocalizer { public boolean valid=true;public boolean isSampleValid(){return valid;}public String getStatus(){return valid?"VALID":"INVALID";} }''',
'com/pedropathing/paths/Path.java': '''package com.pedropathing.paths;
import com.pedropathing.geometry.BezierLine;
public class Path {public double t,heading;public PathConstraints constraints;public Path(BezierLine l,PathConstraints c){constraints=c;}public void setConstantHeadingInterpolation(double h){heading=h;}public double getClosestPointTValue(){return t;}}
''',
'com/pedropathing/follower/Follower.java': '''package com.pedropathing.follower;
import com.pedropathing.geometry.Pose;import com.pedropathing.math.Vector;import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.*;import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.pedroPathing.SafeThreeWheelLocalizer;
public class Follower {
 public static Follower instance;public final SafeThreeWheelLocalizer localizer=new SafeThreeWheelLocalizer();
 public Pose pose=new Pose();public double speed,angular,turnDelta;public boolean busy,throwStop;
 public int updates,outputUpdates,stopCalls,pathCalls,turnCalls;public Path path;public double power;
 private final HardwareMap map;private final Consumer<Pose> validator;
 public Follower(HardwareMap m,Consumer<Pose> v){instance=this;map=m;validator=v;}
 public static class Tracker {private final SafeThreeWheelLocalizer l;Tracker(SafeThreeWheelLocalizer l){this.l=l;}public SafeThreeWheelLocalizer getLocalizer(){return l;}}
 public Tracker getPoseTracker(){return new Tracker(localizer);}
 public void setStartingPose(Pose p){pose=p;}
 public void update(){updates++;if(!localizer.valid)throw new IllegalStateException("invalid localizer");validator.accept(pose);if(busy){outputUpdates++;for(String n:new String[]{"frontLeft","frontRight","backLeft","backRight"})map.get(DcMotorEx.class,n).setPower(power);}}
 public Pose getPose(){return pose;}public Vector getVelocity(){return new Vector(speed,0);}public double getAngularVelocity(){return angular;}
 public boolean isLocalizationNAN(){return !localizer.valid;}public boolean isBusy(){return busy;}
 public void setMaxPower(double p){power=p;}public void followPath(Path p,boolean hold){path=p;pathCalls++;busy=true;}
 public void turn(double delta){turnDelta=delta;turnCalls++;busy=true;}
 public void breakFollowing(){stopCalls++;if(throwStop)throw new IllegalStateException("follower stop");busy=false;for(String n:new String[]{"frontLeft","frontRight","backLeft","backRight"})map.get(DcMotorEx.class,n).setPower(0);}
}''',
'org/firstinspires/ftc/teamcode/pedroPathing/Constants.java': '''package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;import com.pedropathing.geometry.Pose;import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;import java.util.function.Consumer;
public class Constants { public static String issue;public static double mecanumMaxPower=.5;
 public static PathConstraints pathConstraints=new PathConstraints(.99,1,1,1);
 public static String getConfigurationIssue(){return issue;}
 public static Follower createFollower(HardwareMap m,Consumer<Pose> v){if(issue!=null)throw new IllegalStateException(issue);return new Follower(m,v);}
}''',
}
with tempfile.TemporaryDirectory(prefix='biobuzz-red-auto-checks-') as directory:
    temp = Path(directory)
    sdk = temp / 'RobotCore.jar'
    with zipfile.ZipFile(sdk_aar) as archive:
        sdk.write_bytes(archive.read('classes.jar'))
    inputs = []
    for name, content in stubs.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        inputs.append(str(path))
    source = project / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
    inputs += [str(source / name) for name in [
        'autonomous/RedAuto.java', 'autonomous/RedAutoPlan.java', 'subsystems/DriveTrain.java',
        'subsystems/Intake.java', 'subsystems/Transfer.java', 'subsystems/FlowerMechanism.java']]
    inputs.append(str(project / 'TeamCode/src/test/host/RedAutoChecks.java'))
    classes = temp / 'classes'
    classpath = os.pathsep.join([str(sdk), str(core)])
    subprocess.run([javac, '--release', '8', '-cp', classpath, '-d', str(classes)] + inputs, check=True)
    subprocess.run([java, '-cp', os.pathsep.join([str(classes), classpath]),
                    'org.firstinspires.ftc.teamcode.autonomous.RedAutoChecks'], check=True)
