package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AryanOuttakeBox {
    private final Telemetry telemetry;
    private final Servo fork;
    private int targetPosition;
    enum Position {
        OPEN,
        MIDDLE,
        CLOSE
    }
    public AryanOuttakeBox(HardwareMap hardwareMap,Telemetry telemetry) {

        this.telemetry = telemetry;

        fork = hardwareMap.get(Servo.class,"fork");
    }
    void goTo(int targetPosition) {
        fork.setPosition(targetPosition);
    }
    public void goTo(Position targetPosition) {
        switch (targetPosition) {
            case OPEN: goTo(-1);
            case MIDDLE: goTo(0);
            case CLOSE: goTo(1);
        }
    }
}
