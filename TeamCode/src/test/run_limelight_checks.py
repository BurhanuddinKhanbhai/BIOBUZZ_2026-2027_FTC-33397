#!/usr/bin/env python3
"""Test production vision + installed FTC 11.1 LLResult parsing; camera transport is fake."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import zipfile

project = Path(__file__).resolve().parents[3]
cache = Path.home() / '.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc'
java_home = Path(os.environ.get('JAVA_HOME', '/Applications/Android Studio.app/Contents/jbr/Contents/Home'))
javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else shutil.which('javac')
java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else shutil.which('java')
# Host JSON runtime only; this adds no Android/project dependency.
json_jar = Path(os.environ.get('JSON_JAR', '/Applications/Android Studio.app/Contents/plugins/grazie/lib/org.json-json.jar'))
if not json_jar.is_file():
    raise SystemExit('Set JSON_JAR to a local JVM org.json implementation (Android android.jar is a stub).')
stubs = {
    'com/qualcomm/robotcore/hardware/HardwareMap.java': '''package com.qualcomm.robotcore.hardware;
public class HardwareMap {
 private Object camera;
 public HardwareMap(Object camera) {this.camera=camera;}
 public <T>T tryGet(Class<T> type,String name) {
  if(!"limelight".equals(name))throw new AssertionError("Wrong hardware name: "+name);
  return type.isInstance(camera)?type.cast(camera):null;
 }
}''',
    'com/qualcomm/hardware/limelightvision/Limelight3A.java': '''package com.qualcomm.hardware.limelightvision;
public class Limelight3A {
 public boolean connected=true,running=false,switchSuccess=true;
 public int reads,switches,starts,stops,pipeline=-1;
 public LLResult latest;
 public boolean pipelineSwitch(int value){switches++;pipeline=value;return switchSuccess;}
 public void start(){starts++;running=true;}
 public void stop(){stops++;running=false;}
 public boolean isRunning(){return running;}
 public boolean isConnected(){return connected;}
 public LLResult getLatestResult(){reads++;return latest;}
}''',
}
with tempfile.TemporaryDirectory(prefix='biobuzz-limelight-checks-') as directory:
    temp = Path(directory)
    jars = [str(json_jar)]
    for artifact in ['Hardware', 'RobotCore']:
        aar = next((cache / artifact / '11.1.0').glob('*/*.aar'))
        jar = temp / (artifact + '.jar')
        with zipfile.ZipFile(aar) as archive:
            jar.write_bytes(archive.read('classes.jar'))
        jars.append(str(jar))
    inputs = []
    for name, content in stubs.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        inputs.append(str(path))
    inputs += [str(project / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/LimelightVision.java'),
               str(project / 'TeamCode/src/test/host/LimelightChecks.java')]
    classes = temp / 'classes'
    subprocess.run([javac, '--release', '8', '-cp', os.pathsep.join(jars), '-d', str(classes)] + inputs, check=True)
    subprocess.run([java, '-cp', os.pathsep.join([str(classes)] + jars),
                    'org.firstinspires.ftc.teamcode.vision.LimelightChecks'], check=True)
