package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OuttakeBox {
    private final Telemetry telemetry;
    private final CRServo box;

    public OuttakeBox(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        box = hardwareMap.get(CRServo.class,"BOXservo");
        box.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake() {
        box.setPower(-1);
    }

    public void outtake() {
        box.setPower(1);
    }

    public void stopSpinning() {
        box.setPower(0);
    }

    public double getPower() {
        return box.getPower();
    }
}
