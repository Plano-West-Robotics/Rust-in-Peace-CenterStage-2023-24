package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeController;

import java.util.Arrays;

@TeleOp(name="Robot-Centric TeleOp")
public class RobotCentricTeleOp extends OpMode {

    Drive drive;
    Lift lift;
    Intake intake;

    OuttakeController control;


    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        control = new OuttakeController(hardwareMap, telemetry, lift);
    }
    @Override
    public void loop() {

        // Lift
        if (gamepad1.dpad_up) {
            lift.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            lift.setPower(-0.3);
        } else {
            lift.setPower(0);
        }

        // Intake - Spin
        if (gamepad1.left_bumper) {
            intake.spinForward();
        } else if (gamepad1.left_trigger > 0) {
            intake.spinBackwards();
        } else {
            intake.stopSpin();
        }


        // Intake - Arm
        if (gamepad1.right_bumper) {
            intake.setArmPower(0.2);
        } else if (gamepad1.right_trigger > 0) {
            intake.setArmPower(-0.2);
        } else {
            intake.setArmPower(0);
        }

        //Arm - toggle
        /** If arm up, then arm down, vice versa
         */
        if(gamepad1.a) {    //GAMEPAD BUTTON IS NOT FINAL. JUST PLACEHOLDER

            if (control.getArmUp()) {
                control.armDown();
            } else {
                control.armUp();
            }

        }
        // Chassis
        drive.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
        telemetry.addData("Slide Motor (SR-Cloned to left)", lift.getEncoderValue());
    }
}