package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeDifferential;

import java.util.Arrays;

@Disabled
@TeleOp(name="Barebones-TeleOp")
public class BareBonesTeleOp extends OpMode {

    Drive drive;


    double driveSpeedMult = 0.8;


    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry, false);


        drive.setSpeed(1);

    }

    @Override
    public void loop(){

            drive.setSpeed(driveSpeedMult);

            drive.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
            telemetry.addData("IMU Orientation", drive.getHeading());
            telemetry.addData("\nDrive Speed", driveSpeedMult);

        }
    }
