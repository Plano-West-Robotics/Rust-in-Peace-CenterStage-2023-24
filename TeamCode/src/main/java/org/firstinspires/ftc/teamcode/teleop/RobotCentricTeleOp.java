package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeController;

import java.util.Arrays;

@TeleOp(name="Robot-Centric TeleOp")
public class RobotCentricTeleOp extends OpMode {

    Drive drive;
    Lift lift;
    Intake intake;

    OuttakeController control;

    double intakeArmTrigger;

    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry, false);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        control = new OuttakeController(hardwareMap, telemetry, lift);

        intakeArmTrigger = 0;

        // Bypass lift check to prevent swinging
        control.arm.goTo(OuttakeArm.Position.DOWN);

        drive.setSpeed(1);

        lift.loadPosition();
    }

    @Override
    public void loop() {
        // Lift
        if (Math.abs(gamepad2.left_stick_y) > 0) lift.update(-gamepad2.left_stick_y * 0.7);
        else lift.update(0.1);

        // Auto release combos --
        if (gamepad2.dpad_up) {
            lift.setManual(false);
            lift.setTargetPositionPreset(Lift.Position.MEDIUM);
        }

        if (!lift.getManual() && control.liftIsUp() && !control.getArmUp()) control.armUp();
        else if (!lift.getManual() && control.liftIsUp() && control.getArmUp()) control.armDown();

        // Intake - Spin
        if (gamepad2.left_bumper) {
            intake.spinForward();
            lift.setPower(-0.1, true);
            control.spinBoxIn();
        } else if (gamepad2.right_bumper) {
            intake.spinBackwards();
        } else {
            intake.stopSpin();
        }

        // Intake - Arm
        if (gamepad2.right_trigger > 0) {
            intake.setTargetPositionPreset(Intake.Position.DOWN);
        } else if (gamepad2.left_trigger > 0) {
            intake.setTargetPositionPreset(Intake.Position.TOP);
        }

        intake.update();

        // Outtake Arm
        if (-gamepad2.right_stick_y > 0) {
            control.armUp();
        } else if (-gamepad2.right_stick_y < 0) {
            control.armDown();
        }

        // Outtake Box
        if (gamepad2.x) {
            control.spinBoxIn();
        } else if (gamepad2.b) {
            control.spinBoxOut();
        } else if (gamepad2.y){
            control.stopBox();
        }

        // Reset Lift Encoder
        if (gamepad2.back) lift.resetEncoder();

        if (gamepad1.back) drive.resetHeading();

        if (gamepad2.dpad_down) lift.setPower(-0.3, true);

        // Reset field yaw
        if (gamepad1.back) {
            drive.resetHeading();
        }

        // Chassis
        drive.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
        telemetry.addData("Slide Motor (SR-Cloned to left)", lift.getEncoderValue());
        telemetry.addData("Intake Arm (ARM)", intake.getCurrentPosition());
        telemetry.addData("Outtake Arm (LA-Cloned to right)", control.getArmUp() ? "UP" : "DOWN");
        telemetry.addData("Outtake Arm Position (LA)", control.arm.leftArm.getPosition());
        telemetry.addData("Outtake Box Direction (BOX)", control.box.getPower() == 0 ? "STOP" : control.box.getPower() == 1 ? "OUT" : "IN");
    }
}