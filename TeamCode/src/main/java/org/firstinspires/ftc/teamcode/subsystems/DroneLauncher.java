package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private final Telemetry telemetry;
    public final Servo droneArm;
    public final Servo droneShoot;
    private final Lift lift;

    public enum Position {
        UP,
        DOWN
    }
    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneArm = hardwareMap.get(Servo.class,"DAservo");
        droneShoot = hardwareMap.get(Servo.class,"DSservo");

        lift = new Lift(hardwareMap, telemetry);
    }

    public void goTo(double targetPosition) {
        droneArm.setPosition(targetPosition);
    }

    public void goTo(Position targetPosition) {
        if (targetPosition == Position.UP) {
            goTo(0.65);
        } else if (targetPosition == Position.DOWN) {
            goTo(1);
        }
    }

    public void shoot() {
        new Thread(() -> {
            goTo(Position.UP);
            try {
                sleep(500);
            } catch (Exception ignored) {}
            droneShoot.setPosition(0.5);
        }).start();
    }
}
