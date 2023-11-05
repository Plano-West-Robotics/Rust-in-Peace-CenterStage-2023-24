package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.Arrays;

@TeleOp(name="Field-Centric TeleOp")
public class FieldCentricTeleOp extends OpMode {

    Lift lift;
    Drive drive;

    @Override
    public void init() {
        lift = new Lift(hardwareMap, telemetry);
        drive = new Drive(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        drive.updateFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Encoders (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));

    }
}