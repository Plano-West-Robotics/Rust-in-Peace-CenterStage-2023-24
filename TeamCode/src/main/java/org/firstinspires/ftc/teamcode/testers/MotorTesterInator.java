package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "MotorTesterInator")
public class MotorTesterInator extends OpMode {
    private DcMotor motor;
    private double speed;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");

    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
