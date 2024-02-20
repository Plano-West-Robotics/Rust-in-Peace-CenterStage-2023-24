package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OuttakeBox {
    private final Telemetry telemetry;
    public final CRServo box;
    public final Servo wrist;
    public final ColorSensor frontSense;
    public final ColorSensor backSense;

    private final int THRESHOLD = 400;

    public enum State {
        P1,
        P2,
        P3,
        P4,
        P5,
        P6,
        P7
    }

    public OuttakeBox(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        box = hardwareMap.get(CRServo.class,"BOXservo");
        box.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class, "WRservo");

        frontSense = hardwareMap.get(ColorSensor.class, "FRsensor");
        backSense = hardwareMap.get(ColorSensor.class, "BAsensor");
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
                    wrist.setPosition(0.01);
                } else if (state == State.P2) {
                    // roller left
                    wrist.setPosition(1);
                } else if (state == State.P3) {
                    // passive
                    wrist.setPosition(0.69); // change
                } else if (state == State.P4) {
                    // roller right
                      wrist.setPosition(0.32);
                } else if (state == State.P5) {
                    // passive left center
                    wrist.setPosition(0.58);
                } else if (state == State.P6) {
                    // passive right center
                    wrist.setPosition(0.81);
                } else if (state == State.P7) {
                    // active left center
                    wrist.setPosition(0.13);
                }
                sleep(1500);
            } catch (Exception ignored) {}
        }).start();
    }

    public void setWristPositionManual(double position) {
        new Thread(() -> {
            wrist.setPosition(position);
        }).start();
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }

    public boolean boxIsFull() {
        if (frontSense.red() > THRESHOLD && frontSense.green() > THRESHOLD && frontSense.blue() > THRESHOLD &&
                backSense.red() > THRESHOLD && backSense.green() > THRESHOLD && backSense.blue() > THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    public boolean boxIsEmpty() {
        if ((frontSense.red() < THRESHOLD || frontSense.green() < THRESHOLD || frontSense.blue() < THRESHOLD) &&
                (backSense.red() < THRESHOLD || backSense.green() < THRESHOLD || backSense.blue() < THRESHOLD)) {
            return true;
        } else {
            return false;
        }
    }
}
