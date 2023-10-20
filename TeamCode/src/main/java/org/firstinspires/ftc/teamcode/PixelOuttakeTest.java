package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;

@TeleOp(name="PixelOuttakeTest", group="Actions")
public class PixelOuttakeTest extends LinearOpMode {

    private OuttakeBox outtakeBox;

    @Override
    public void runOpMode() throws InterruptedException {

        outtakeBox = new OuttakeBox(hardwareMap, telemetry);

        waitForStart();

        outtakeBox.release();
        outtakeBox.release();

    }
}
