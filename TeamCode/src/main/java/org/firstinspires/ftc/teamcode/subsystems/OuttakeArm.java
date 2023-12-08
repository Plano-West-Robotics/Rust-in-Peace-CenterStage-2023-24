package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeArm {
    private final Telemetry telemetry;
    public final Servo leftArm;

    final double leftPos;

    public enum Position {
        UP,
        DOWN
    }
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftArm = hardwareMap.get(Servo.class,"LAservo");

        leftArm.scaleRange(0, 0.6);

        leftPos = leftArm.getPosition();
    }

    public void goTo(double targetPosition) {
        leftArm.setPosition(targetPosition);
    }

    public void goTo(Position targetPosition) {
        if (targetPosition == Position.UP) {
            goTo(0.8);
        } else if (targetPosition == Position.DOWN) {
            goTo(0.03);
        }
    }
}
