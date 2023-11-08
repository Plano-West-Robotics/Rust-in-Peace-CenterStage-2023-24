package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.Arrays;

@TeleOp(name="Field-Centric TeleOp")
public class FieldCentricTeleOp extends OpMode {

    Drive drive;
    Lift lift;
    Intake intake;

    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // Chassis
        drive.updateFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
        telemetry.addData("Slide Motor (SR-Cloned to left)", lift.getEncoderValue());
        telemetry.addData("Intake Arm (ARM)", intake.getCurrentPosition());
    }
}