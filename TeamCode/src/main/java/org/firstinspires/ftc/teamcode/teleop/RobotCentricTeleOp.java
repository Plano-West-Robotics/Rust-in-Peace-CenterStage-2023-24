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
        drive = new Drive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        control = new OuttakeController(hardwareMap, telemetry, lift);

        intakeArmTrigger = 0;

        // Bypass lift check to prevent swinging
        control.arm.goTo(OuttakeArm.Position.DOWN);
    }

    @Override
    public void loop() {

        // test variable lift

        // Lift
        if (Math.abs(gamepad2.left_stick_y) > 0) lift.update(-gamepad2.left_stick_y);
        else lift.update(0.1);

        // Auto release combo
        if (gamepad2.a) {
            lift.setManual(false);
            lift.setTargetPositionPreset(Lift.Position.MEDIUM);
        }

        if (!lift.getManual() && control.liftIsUp() && !control.getArmUp()) control.armUp();

        // Intake - Spin
        if (gamepad2.left_bumper) {
            intake.spinForward();
        } else if (gamepad2.left_trigger > 0) {
            intake.spinBackwards();
        } else {
            intake.stopSpin();
        }


        // Intake - Arm
        if (gamepad2.right_trigger < intakeArmTrigger) {
            if (intakeArmTrigger > 0 && intakeArmTrigger < 0.7) intake.setTargetPositionPreset(Intake.Position.MIDDLE);
        } else if (gamepad2.right_trigger > intakeArmTrigger) {
            intakeArmTrigger = gamepad2.right_trigger;
        }

        if (gamepad2.right_trigger < 0.1){
            intakeArmTrigger = 0;
        }

        if (gamepad2.right_bumper) {
            intake.setTargetPositionPreset(Intake.Position.TOP);
        } else if (gamepad2.right_trigger > 0.9) {
            intake.setTargetPositionPreset(Intake.Position.DOWN);
        }

        intake.update();

        // Outtake Arm
        if(gamepad2.dpad_left) {
            control.armUp();
        } else if (gamepad2.dpad_right) {
            control.armDown();
        }

        if (control.getArmUp()) control.armUp();
        else control.armDown();

        // Outtake Box
        if (gamepad2.b) {
            control.spinBoxIn();
        } else if (gamepad2.x) {
            control.spinBoxOut();
        } else if (gamepad2.y){
            control.stopBox();
        }

        // Reset Lift Encoder
        if (gamepad2.back) lift.resetEncoder();

        // Chassis
        drive.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
        telemetry.addData("Slide Motor (SR-Cloned to left)", lift.getEncoderValue());
        telemetry.addData("Intake Arm (ARM)", intake.getCurrentPosition());
        telemetry.addData("Outtake Arm (LA-Cloned to right)", control.getArmUp() ? "UP" : "DOWN");
        telemetry.addData("Outtake Arm Position (L-R)", control.arm.leftArm.getPosition() + " " + control.arm.rightArm.getPosition());
        telemetry.addData("Outtake Box Direction (BOX)", control.box.getPower() == 0 ? "STOP" : control.box.getPower() == 1 ? "OUT" : "IN");
    }
}