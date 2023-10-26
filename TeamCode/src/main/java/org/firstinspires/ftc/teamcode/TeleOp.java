package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
class TeleOp extends OpMode {
    private Drive drive;

    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry);
        drive.setSpeed(0.5);

    }

    @Override
    public void loop() {
        drive.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }
}