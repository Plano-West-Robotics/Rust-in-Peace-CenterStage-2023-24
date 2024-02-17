package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private final Telemetry telemetry;
    public final Servo droneArm; //can remove this?
    public final Servo droneShoot;
    private final Lift lift;

    public enum Position {
        UP,
        DOWN
    }
    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneArm = hardwareMap.get(Servo.class,"DAservo"); //can remove this?
        droneShoot = hardwareMap.get(Servo.class,"DSservo");

        lift = new Lift(hardwareMap, telemetry);
    }

    public void goTo(double targetPosition) {
        droneArm.setPosition(targetPosition);
    } //can remove this?

    public void goTo(Position targetPosition) { //can remove this?
        if (targetPosition == Position.UP) {
            goTo(0.65);
        } else if (targetPosition == Position.DOWN) {
            goTo(1);
        }
    }

    public void shoot() {
        new Thread(() -> {
            goTo(Position.UP); //can remove this?
            try {
                sleep(500);
            } catch (Exception ignored) {}
            droneShoot.setPosition(0.5);
        }).start();
    }
}
