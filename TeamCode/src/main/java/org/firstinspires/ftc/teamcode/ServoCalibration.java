package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Direct, independently enabled servo targets from Panels Configurables. */
@Configurable
@TeleOp(name = "TEST - Servo Calibration", group = "Testing")
public class ServoCalibration extends OpMode {
    // UI targets, not automatic initialization commands or calibrated endpoints.
    public static double flowerPosition = 0.50;
    public static double flower2Position = 0.50;
    public static double shooterPosition = 0.50;
    public static boolean flowerEnabled = false;
    public static boolean flower2Enabled = false;
    public static boolean shooterEnabled = false;

    private Servo flowerServo;
    private Servo flowerServo2;
    private Servo shooterServo;
    private double flowerLastCommand = Double.NaN;
    private double flower2LastCommand = Double.NaN;
    private double shooterLastCommand = Double.NaN;

    @Override
    public void init() {
        flowerServo = hardwareMap.tryGet(Servo.class, "flowerServo");
        flowerServo2 = hardwareMap.tryGet(Servo.class, "flowerServo2");
        shooterServo = hardwareMap.tryGet(Servo.class, "shooterAdjust");
        showTelemetry();
    }

    @Override
    public void start() {
        if (flowerServo2 != null) flowerServo2.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (flowerEnabled && flowerServo != null && flowerPosition != flowerLastCommand) {
            flowerServo.setPosition(flowerPosition);
            flowerLastCommand = flowerPosition;
        }
        if (flower2Enabled && flowerServo2 != null && flower2Position != flower2LastCommand) {
            flowerServo2.setPosition(flower2Position);
            flower2LastCommand = flower2Position;
        }
        if (shooterEnabled && shooterServo != null && shooterPosition != shooterLastCommand) {
            shooterServo.setPosition(shooterPosition);
            shooterLastCommand = shooterPosition;
        }
        showTelemetry();
    }

    @Override
    public void stop() {
        flowerEnabled = flower2Enabled = shooterEnabled = false;
        // STOP does not command a different physical position.
    }

    private void showTelemetry() {
        telemetry.addLine("Panels: enable a servo and set its target; changes apply immediately.");
        telemetry.addData("Flower 1 mapped / enabled", "%s / %s", flowerServo != null, flowerEnabled);
        telemetry.addData("Flower 1 target / commanded", "%.3f / %.3f", flowerPosition, flowerLastCommand);
        telemetry.addData("Flower 2 mapped / enabled", "%s / %s", flowerServo2 != null, flower2Enabled);
        telemetry.addData("Flower 2 target / commanded", "%.3f / %.3f", flower2Position, flower2LastCommand);
        telemetry.addData("Shooter adjust mapped / enabled", "%s / %s", shooterServo != null, shooterEnabled);
        telemetry.addData("Shooter target / commanded", "%.3f / %.3f", shooterPosition, shooterLastCommand);
        telemetry.addLine("Commanded targets are not physical position feedback. Flower servo 2 is REVERSE.");
        telemetry.update();
    }
}
