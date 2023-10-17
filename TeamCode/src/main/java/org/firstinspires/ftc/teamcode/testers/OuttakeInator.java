package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;

@TeleOp(name="OuttakeInator", group="Testers")
public class OuttakeInator extends OpMode
{
    private OuttakeBox outtakeBox;

    @Override
    public void init() {
        outtakeBox = new OuttakeBox(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        outtakeBox.update();
    }
}
