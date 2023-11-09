package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeArm {
    private final Telemetry telemetry;
    public final Servo leftArm;
    public final Servo rightArm;

    final double leftPos;
    final double rightPos;

    public enum Position {
        UP,
        DOWN
    }
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftArm = hardwareMap.get(Servo.class,"LAservo");
        rightArm = hardwareMap.get(Servo.class, "RAservo");

        leftArm.scaleRange(0, 0.45);
        rightArm.setDirection(Servo.Direction.REVERSE);

        leftPos = leftArm.getPosition();
        rightPos = rightArm.getPosition();
    }

    public void goTo(double targetPosition) {
        leftArm.setPosition(targetPosition);
    }

    public void goTo(Position targetPosition) {
        if (targetPosition == Position.UP) {
            goTo(1);
        } else if (targetPosition == Position.DOWN) {
            goTo(0);
        }
    }
}
