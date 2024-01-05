package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private final Telemetry telemetry;
    public final Servo droneArm;
    private final Servo droneShoot;
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
            goTo(0.9);
        }
    }

    public void shoot() throws InterruptedException {
        lift.update(0.8);
        sleep(400);
        lift.update(0.1);
        goTo(Position.UP);
        sleep(500);
        droneShoot.setPosition(0.5);
    }
}
