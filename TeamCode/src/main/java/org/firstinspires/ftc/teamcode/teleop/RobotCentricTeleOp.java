package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeDifferential;

import java.util.Arrays;

@TeleOp(name="Robot-Centric TeleOp")
public class RobotCentricTeleOp extends OpMode {

    Drive drive;
    Lift lift;
    Intake intake;
    DroneLauncher droneLauncher;

    OuttakeDifferential outtake;

    double intakeArmTrigger;
    double driveSpeedMult = 0.8;
    double liftSpeedMult = 1;

    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry, false);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        droneLauncher = new DroneLauncher(hardwareMap, telemetry);

        outtake = new OuttakeDifferential(hardwareMap, telemetry, OuttakeDifferential.State.DOWN);

        drive.setSpeed(1);
        lift.loadPosition();
        lift.setManual(true);
    }

    @Override
    public void loop() {
        try {
            // Lift
            if (Math.abs(gamepad2.left_stick_y) > 0)
                lift.update(-gamepad2.left_stick_y * liftSpeedMult);
            else lift.update(0.1);

            // Intake - Spin
            if (gamepad2.left_bumper) {
                intake.spinForward();
                lift.setPower(-0.1, true);
                outtake.box.stopSpinning();
                outtake.box.intake();
            } else if (gamepad2.right_bumper) {
                intake.spinBackwards();
            } else {
                intake.stopSpin();
            }

            // Intake - Arm
//        if (gamepad2.right_trigger > 0) {
//            intake.setTargetPositionPreset(Intake.Position.DOWN);
//        } else if (gamepad2.left_trigger > 0) {
//            intake.setTargetPositionPreset(Intake.Position.TOP);
//        }
//
//        intake.update();

            // Outtake Arm
            if (-gamepad2.right_stick_y > 0.8) {
                outtake.goTo(OuttakeDifferential.State.UP);
            } else if (-gamepad2.right_stick_y < -0.8) {
                outtake.goTo(OuttakeDifferential.State.DOWN);
            }

            if (gamepad2.right_stick_x > 0.8) {
                outtake.setWrist(OuttakeDifferential.WristState.MANUAL);
            } else if (gamepad2.right_stick_x < -0.8) {
                outtake.setWrist(OuttakeDifferential.WristState.PASSIVE);
            }

            // Outtake Box
            if (gamepad2.x) {
                outtake.box.intake();
            } else if (gamepad2.b) {
                outtake.box.outtake();
            } else if (gamepad2.y) {
                outtake.box.stopSpinning();
            }

            // Reset Lift Encoder
            if (gamepad2.back) lift.resetEncoder();

            if (gamepad2.dpad_down) lift.setPower(-0.3, true);

            // Drone
            if (gamepad1.y) {
                droneLauncher.shoot();
            }

            if (gamepad1.x) droneLauncher.goTo(DroneLauncher.Position.DOWN);
            else if (gamepad1.b) droneLauncher.goTo(DroneLauncher.Position.UP);

            // Chassis
            if (gamepad1.left_bumper) driveSpeedMult -= (driveSpeedMult - 0.1 < 0 ? 0 : 0.1);
            else if (gamepad1.right_bumper) driveSpeedMult += (driveSpeedMult + 0.1 > 1 ? 0 : 0.1);

            if (gamepad1.back) drive.resetHeading();

            drive.setSpeed(driveSpeedMult);

            drive.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
            telemetry.addData("Slide Motor (SR-Cloned to left)", lift.getEncoderValue());
            telemetry.addData("Intake Arm (ARM)", intake.getCurrentPosition());
            telemetry.addData("Outtake Arm", outtake.state);
            telemetry.addData("Wrist Outtake Direction", outtake.wristState);
            telemetry.addData("Outtake Box Direction (BOX)", outtake.box.getPower() == 0 ? "STOP" : outtake.box.getPower() == 1 ? "OUT" : "IN");
            telemetry.addData("IMU Orientation", drive.getHeading());
            telemetry.addData("\nDrive Speed", driveSpeedMult);
            telemetry.addData("Lift Speed", liftSpeedMult);
        } catch (Exception ignored) {}
    }
}