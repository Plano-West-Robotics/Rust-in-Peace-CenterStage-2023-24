package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeArm {
    private final Telemetry telemetry;
    private final Servo arm;
    public enum Position {
        UP,
        DOWN
    }
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        arm = hardwareMap.get(Servo.class,"arm");
    }

    public void goTo(double targetPosition) {
        arm.setPosition(targetPosition);
    }

    public void goTo(Position targetPosition) {
        switch (targetPosition) {
            case UP: goTo(-1);   // NOT FINAL
            case DOWN: goTo(0);  // NOT FINAL
        }
    }
}
