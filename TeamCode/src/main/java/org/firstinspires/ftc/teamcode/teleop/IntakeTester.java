package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeController;

import java.util.Arrays;

@TeleOp(name="Intake Tester")
public class IntakeTester extends OpMode {

    Intake intake;

    double intakeArmTrigger;
    double speed;
    int arm = 1;

    @Override
    public void init() {
        intake = new Intake(hardwareMap, telemetry);
        intakeArmTrigger = 0;

        speed = 0.5;
    }

    @Override
    public void loop() {
        speed = Math.abs(gamepad1.left_stick_y)*0.8;
        intake.setSpeed(speed);

        if (gamepad1.a) intake.spinBackwards();
        else intake.spinForward();

        if (gamepad1.y) intake.setTargetPositionPreset(Intake.Position.TOP);
        else if (gamepad1.dpad_up) intake.setTargetPositionPreset(Intake.Position.P5);
        else if (gamepad1.dpad_left) intake.setTargetPositionPreset(Intake.Position.P4);
        else if (gamepad1.dpad_right) intake.setTargetPositionPreset(Intake.Position.P3);
        else if (gamepad1.dpad_down) intake.setTargetPositionPreset(Intake.Position.P2);
        else if (gamepad1.b) intake.setTargetPositionPreset(Intake.Position.DOWN);

        intake.update();

        telemetry.addData("Speed", speed);
        telemetry.addData("Position", intake.getCurrentPosition());
        telemetry.update();
    }
}