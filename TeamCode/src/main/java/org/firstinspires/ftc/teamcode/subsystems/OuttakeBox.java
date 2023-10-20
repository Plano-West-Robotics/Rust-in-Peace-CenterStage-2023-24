package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OuttakeBox {
    private final Telemetry telemetry;
    private final Servo fork;
    public static double targetPosition;
    enum Position {
        OPEN,
        MIDDLE,
        CLOSE
    }
    public OuttakeBox(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        fork = hardwareMap.get(Servo.class,"fork");
    }

    public void goTo(double targetPosition) {
        fork.setPosition(targetPosition);
    }
    public void goTo(Position targetPosition) {
        switch (targetPosition) {
            case OPEN: goTo(-1);
            case MIDDLE: goTo(0);
            case CLOSE: goTo(1);
        }
    }

    public void release() throws InterruptedException {
        this.goTo(0.55);
        sleep(400);
        this.goTo(0.4);
        sleep(400);
        this.goTo(0.55);
    }

    public void update() {
        fork.setPosition(targetPosition);
    }
}
