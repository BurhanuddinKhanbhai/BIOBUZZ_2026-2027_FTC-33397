package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/** Robot-centric driving on Gamepad 1 and manual intake/transfer controls on Gamepad 2. */
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class Teleop extends OpMode {
    // Initial limits for an untested robot; tune within [0.0, 1.0] after hardware validation.
    private static final double NORMAL_SPEED_SCALE = 0.40;
    private static final double PRECISION_SPEED_SCALE = 0.20;
    // Per-axis threshold in [0.0, 1.0); inputs outside it retain their original magnitude.
    private static final double JOYSTICK_DEADZONE = 0.05;
    private static final double INTAKE_POWER = 0.35;
    private static final double TRANSFER_POWER = 0.35;
    // Reverse activates while the corresponding trigger exceeds this threshold.
    private static final float TRIGGER_THRESHOLD = 0.20f;

    private DriveTrain driveTrain;
    private Intake intake;
    private Transfer transfer;

    @Override
    public void init() {
        // Retain DriveTrain's motor directions, BRAKE behavior, and encoder-safe motor modes.
        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.setSpeedScale(NORMAL_SPEED_SCALE);
        intake = new Intake(hardwareMap, DcMotorSimple.Direction.FORWARD, INTAKE_POWER);
        transfer = new Transfer(hardwareMap, DcMotorSimple.Direction.FORWARD, TRANSFER_POWER);
        telemetry.addData("Status", "Drivetrain, intake, and transfer initialized");
        telemetry.addData("Speed scale", driveTrain.getSpeedScale());
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean precision = gamepad1.left_bumper;
        driveTrain.setSpeedScale(precision ? PRECISION_SPEED_SCALE : NORMAL_SPEED_SCALE);

        // Positive commands move forward, strafe right, and turn clockwise from above.
        double forward = applyDeadzone(-gamepad1.left_stick_y);
        double strafe = applyDeadzone(gamepad1.left_stick_x);
        double turn = applyDeadzone(gamepad1.right_stick_x);
        // Gamepad 1 right bumper is reserved for future hold-to-aim; it has no effect here.
        driveTrain.drive(forward, strafe, turn);

        // Hold-to-run; reverse takes priority independently for each mechanism.
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            intake.reverse();
        } else if (gamepad2.a || gamepad2.x) {
            intake.intake();
        } else {
            intake.stop();
        }

        // Manual transfer can feed directly into the shooter; there is no readiness interlock.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            transfer.reverse();
        } else if (gamepad2.b || gamepad2.x) {
            transfer.feed();
        } else {
            transfer.stop();
        }

        telemetry.addData("Drive command", "Forward %.2f | Strafe %.2f | Turn %.2f",
                forward, strafe, turn);
        telemetry.addData("Drive mode", precision ? "PRECISION" : "NORMAL");
        telemetry.addData("Speed scale", driveTrain.getSpeedScale());
        telemetry.addData("Front power", "Left %.2f | Right %.2f",
                driveTrain.getFrontLeftPower(), driveTrain.getFrontRightPower());
        telemetry.addData("Rear power", "Left %.2f | Right %.2f",
                driveTrain.getBackLeftPower(), driveTrain.getBackRightPower());
        telemetry.addData("Drive state", driveTrain.getState());
        telemetry.addData("Intake state", intake.getState());
        telemetry.addData("Intake power", intake.getPower());
        telemetry.addData("Transfer state", transfer.getState());
        telemetry.addData("Transfer power", transfer.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (driveTrain != null) {
            driveTrain.stop();
        }
        if (intake != null) {
            intake.stop();
        }
        if (transfer != null) {
            transfer.stop();
        }
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) <= JOYSTICK_DEADZONE ? 0.0 : value;
    }
}
