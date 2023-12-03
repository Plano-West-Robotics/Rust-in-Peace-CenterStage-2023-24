package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeController;

import java.util.Arrays;

@Disabled
@TeleOp(name="Outtake Tester")
public class OuttakeTester extends OpMode {

    OuttakeBox box;

    @Override
    public void init() {
        box = new OuttakeBox(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            box.box.setPower(1);
        } else {
            box.box.setPower(-1);
        }

        telemetry.addData("Power", box.getPower());
        telemetry.update();
    }
}