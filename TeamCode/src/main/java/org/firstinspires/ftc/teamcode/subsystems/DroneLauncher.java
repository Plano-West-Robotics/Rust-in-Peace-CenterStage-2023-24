package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private final Telemetry telemetry;
    private final Servo droneArm;
    private final Servo droneShoot;

    public enum Position {
        UP,
        DOWN
    }
    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneArm = hardwareMap.get(Servo.class,"DAservo");
        droneShoot = hardwareMap.get(Servo.class,"DSservo");
    }

    public void goTo(double targetPosition) {
        droneArm.setPosition(targetPosition);
    }

    public void goTo(Position targetPosition) {
        if (targetPosition == Position.UP) {
            goTo(0.6);
        } else if (targetPosition == Position.DOWN) {
            goTo(0);
        }
    }

    public void shoot() {
        droneShoot.setPosition(0.5);
    }
}
