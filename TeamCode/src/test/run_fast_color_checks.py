#!/usr/bin/env python3
"""Host-only checks using installed FTC interfaces and small Android/OpMode stand-ins. No hardware."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import zipfile

project = Path(__file__).resolve().parents[3]
cache = Path.home() / '.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/RobotCore/11.1.0'
aar = next(cache.glob('*/*.aar'))
java_home = Path(os.environ.get('JAVA_HOME', '/Applications/Android Studio.app/Contents/jbr/Contents/Home'))
javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else shutil.which('javac')
java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else shutil.which('java')
stubs = {
    'com/bylazar/configurables/annotations/Configurable.java': '''package com.bylazar.configurables.annotations;
public @interface Configurable {}''',
    'android/content/Context.java': '''package android.content;
public class Context { private final java.io.File files; public Context(java.io.File f){files=f;}
public java.io.File getFilesDir(){return files;} }''',
    'com/qualcomm/robotcore/hardware/HardwareMap.java': '''package com.qualcomm.robotcore.hardware;
public class HardwareMap {
 public android.content.Context appContext;
 private final java.util.Map<String,Object> devices=new java.util.HashMap<>();
 public HardwareMap(java.io.File directory){appContext=new android.content.Context(directory);}
 public void put(String name,Object device){devices.put(name,device);}
 public <T>T tryGet(Class<T> type,String name){Object d=devices.get(name); return type.isInstance(d)?type.cast(d):null;}
 public <T>T get(Class<T> type,String name){T d=tryGet(type,name);if(d==null)throw new IllegalArgumentException(name);return d;}
}''',
    'com/qualcomm/robotcore/hardware/Gamepad.java': '''package com.qualcomm.robotcore.hardware;
public class Gamepad {
 public boolean a,b,x,y,left_bumper,right_bumper,dpad_up,dpad_down,dpad_left,dpad_right;
 public float left_stick_y,left_stick_x,right_stick_x,left_trigger,right_trigger;
}''',
    'com/qualcomm/robotcore/eventloop/opmode/OpMode.java': '''package com.qualcomm.robotcore.eventloop.opmode;
public abstract class OpMode {
 public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
 public org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
 public com.qualcomm.robotcore.hardware.Gamepad gamepad1=new com.qualcomm.robotcore.hardware.Gamepad();
 public com.qualcomm.robotcore.hardware.Gamepad gamepad2=new com.qualcomm.robotcore.hardware.Gamepad();
 public abstract void init(); public abstract void loop(); public void start(){} public void stop(){}
}''',
    'com/qualcomm/robotcore/util/RobotLog.java': '''package com.qualcomm.robotcore.util;
public class RobotLog { public static void ee(String tag,Throwable e,String text){throw new AssertionError(text,e);} }''',
    'org/firstinspires/ftc/teamcode/vision/LimelightVision.java': '''package org.firstinspires.ftc.teamcode.vision;
public class LimelightVision {
public double tx=Double.NaN,ty=Double.NaN; public int updates; public boolean stopped;
public LimelightVision(com.qualcomm.robotcore.hardware.HardwareMap h,int p,long age){}
public void start(){} public void stop(){stopped=true;} public void update(){updates++;}
public boolean hasTarget(){return Double.isFinite(tx)&&Double.isFinite(ty);}
public double getTx(){return tx;} public double getTy(){return ty;}
public double getResultAgeMillis(){return 0;} public int getPipelineIndex(){return 0;}
public String getStatus(){return hasTarget()?"TARGET":"NO_TARGET";} }''',
}
with tempfile.TemporaryDirectory(prefix='biobuzz-color-checks-') as directory:
    temp = Path(directory)
    sdk = temp / 'RobotCore.jar'
    with zipfile.ZipFile(aar) as archive:
        sdk.write_bytes(archive.read('classes.jar'))
    inputs = []
    for name, content in stubs.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        inputs.append(str(path))
    source = project / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
    inputs += [str(source / name) for name in [
        'sensors/BallColorCalibration.java', 'sensors/BallColorSensor.java',
        'subsystems/Shooter.java', 'subsystems/DriveTrain.java', 'subsystems/Intake.java',
        'subsystems/Transfer.java', 'subsystems/FlowerMechanism.java',
        'robot/ShootingCoordinator.java', 'teleOp/Teleop.java', 'FastColorTest.java', 'ServoCalibration.java']]
    # Keep host fixtures outside src/test/java: they intentionally use Android stand-ins,
    # and must not be mistaken for Android Gradle/JUnit tests.
    inputs.append(str(project / 'TeamCode/src/test/host/FastColorChecks.java'))
    classes = temp / 'classes'
    subprocess.run([javac, '--release', '8', '-cp', str(sdk), '-d', str(classes)] + inputs, check=True)
    subprocess.run([java, '-cp', os.pathsep.join([str(classes), str(sdk)]),
                    'org.firstinspires.ftc.teamcode.sensors.FastColorChecks', str(temp)], check=True)
