package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Single Motor Test", group="Testers")
public class SingleMotorTest extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testMotor;
    private DcMotor testMotor2;

    private static final double testMotorPower = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Powered On");

        testMotor = hardwareMap.get(DcMotor.class, "SLmotor");
        testMotor2 = hardwareMap.get(DcMotor.class, "SRmotor");

        testMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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
        testMotor2.setPower(testMotorPower);

        telemetry.addData("Motor Power", "%.2f", testMotorPower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {
    }

}
