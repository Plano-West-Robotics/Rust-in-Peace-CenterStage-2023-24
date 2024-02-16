package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private final Telemetry telemetry;
    public final Servo droneShoot;
    private final Lift lift;

    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneShoot = hardwareMap.get(Servo.class,"DSservo");

        lift = new Lift(hardwareMap, telemetry);
    }

    public void shoot() {
        new Thread(() -> {
            try {
                sleep(500);
            } catch (Exception ignored) {}
            droneShoot.setPosition(0.5);
        }).start();
    }
}
