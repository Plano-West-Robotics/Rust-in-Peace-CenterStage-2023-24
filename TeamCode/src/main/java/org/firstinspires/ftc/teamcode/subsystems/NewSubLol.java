package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewSubLol {
    private final Telemetry tele;

    private final DcMotor fLeft;
    private final DcMotor fRight;
    private final DcMotor bLeft;
    private final DcMotor bRight;

    public NewSubLol(OpMode ops){
        this(ops.telemetry, ops.hardwareMap);
    }

    public NewSubLol(Telemetry tele, HardwareMap hw)
    {
        this.tele = tele;

        tele.addData("Status", "Initializing motors");

        fLeft = hw.get(DcMotor.class, "front_left");
        fRight = hw.get(DcMotor.class, "front_right");
        bLeft = hw.get(DcMotor.class, "back_left");
        bRight = hw.get(DcMotor.class, "back_right");

        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double x, double y, double t)
    {
        double fLP = y + x - t;
        double fRP = y - x + t;
        double bLP = y - x - t;
        double bRP = y + x + t;

        fLeft.setPower(fLP);
        bLeft.setPower(bLP);
        fRight.setPower(fRP);
        bRight.setPower(bRP);
    }
}
