package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private final Telemetry telemetry;
    public final Servo droneShoot;
    private final Lift lift;

    public enum Position {
        UP,
        DOWN
    }
    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneShoot = hardwareMap.get(Servo.class,"DSservo");

        lift = new Lift(hardwareMap, telemetry);
    }

    public void shoot() {
        new Thread(() -> {
            droneShoot.setPosition(0.6);
        }).start();
    }
}
