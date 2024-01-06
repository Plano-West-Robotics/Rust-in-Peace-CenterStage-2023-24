package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OuttakeBox {
    private final Telemetry telemetry;
    public final CRServo box;
    public final Servo wrist;

    public enum State {
        P1,
        P2,
        P3,
        P4
    }

    public OuttakeBox(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        box = hardwareMap.get(CRServo.class,"BOXservo");
        box.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class, "WRservo");
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

    public void setWristPosition(State state) {
        new Thread(() -> {
            try {
                if (state == State.P1) {
                    // manual
                    wrist.setPosition(0);
                } else if (state == State.P2) {
                    // roller left
                    wrist.setPosition(1);
                } else if (state == State.P3) {
                    // passive
                    wrist.setPosition(0.62);
                } else if (state == State.P4) {
                    // roller right
                    wrist.setPosition(0.33);
                }
                sleep(1500);
            } catch (Exception ignored) {}
        }).start();
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }
}
