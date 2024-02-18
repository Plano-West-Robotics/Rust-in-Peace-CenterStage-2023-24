package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name="Back Left", group="Testers")
public class SingleMotorTestLeftBack extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testMotor;
    private DcMotor testMotor2;

    private static final double testMotorPower = 1;

    @Override
    public void init() {
        telemetry.addData("Status", "Powered On");

        testMotor = hardwareMap.get(DcMotor.class, "BLmotor");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        testMotor.setPower(testMotorPower);

        telemetry.addData("Motor Power", "%.2f", testMotorPower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {
    }

}
