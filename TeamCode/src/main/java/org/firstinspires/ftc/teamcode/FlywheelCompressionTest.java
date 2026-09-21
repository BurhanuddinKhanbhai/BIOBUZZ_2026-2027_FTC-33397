package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controller 1: right trigger controls flywheel power; D-pad left/right retracts/compresses.
 * Configure a DC motor named "flywheel" and a positional servo named "move1".
 */
@TeleOp(name = "Flywheel Compression Test", group = "Test")
public class FlywheelCompressionTest extends LinearOpMode {
    // Starting positions only: tune these within 0.0-1.0 to suit your linkage.
    private static final double RETRACTED_POSITION = 0.35;
    private static final double COMPRESSED_POSITION = 0.65;

    // Lower this (for example, to 0.5) to limit power during testing.
    private static final double MAX_FLYWHEEL_POWER = 1.0;

    @Override
    public void runOpMode() {
        DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        Servo move1 = hardwareMap.get(Servo.class, "move1");

        flywheel.setPower(0.0);
        // Change FORWARD to REVERSE if the flywheel spins the wrong way.
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        // Open-loop power control; no motor encoder connection is required.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        move1.setDirection(Servo.Direction.FORWARD);
        double servoPosition = RETRACTED_POSITION;
        // INIT moves the servo to the retracted position.
        move1.setPosition(servoPosition);

        telemetry.addLine("Ready: controller 1");
        telemetry.addLine("Right trigger: flywheel power (release to coast)");
        telemetry.addLine("D-pad left: retract | D-pad right: compress");
        telemetry.addData("move1 target", "%.2f", servoPosition);
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {
                // Trigger travel maps directly to power, not a regulated RPM target.
                double flywheelPower = gamepad1.right_trigger * MAX_FLYWHEEL_POWER;
                flywheel.setPower(flywheelPower);

                // Hold the last position when neither (or both) direction is pressed.
                if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                    servoPosition = RETRACTED_POSITION;
                } else if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                    servoPosition = COMPRESSED_POSITION;
                }
                move1.setPosition(servoPosition);

                telemetry.addData("Flywheel power", "%.0f%%", flywheelPower * 100.0);
                telemetry.addData("move1 target", "%.2f", servoPosition);
                telemetry.addLine("Right trigger: spin | D-pad left/right: retract/compress");
                telemetry.update();
                idle();
            }
        } finally {
            // Remove motor power on STOP, including an interrupted run.
            flywheel.setPower(0.0);
        }
    }
}
