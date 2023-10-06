package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AryanLift {
    private final Telemetry telemetry;

    private final DcMotor lift;

    private boolean manual;

    private double speed;

    private int targetPosition;

    enum Position {
        BOTTOM,
        MEDIUM,
        HIGH
    }
    // THIS IS A CONSTRUCTOR
    public AryanLift(HardwareMap hardwareMap, Telemetry telemetry) {

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        manual = false;

        speed = 1;

        setTargetPositionPreset(Position.BOTTOM);

        this.telemetry = telemetry;
    }

    public void setManual(boolean manual) {
        this.manual = manual;
    }

    public void setPower(double power) {
        lift.setPower(power * speed);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setTargetPositionPreset(Position targetPosition) {
        switch (targetPosition) {
            case BOTTOM: setTargetPosition(-15); break; // Generally values from 0 to 1000
            case MEDIUM: setTargetPosition(500); break;
            case HIGH: setTargetPosition(1000); break;
        }
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double calculate(int currentPosition) {
        int distance = Math.abs(targetPosition - currentPosition);
        if (distance < 20) {
            return 0.01;
        } else if (distance < 100) {
            if (targetPosition < currentPosition) {
                return -0.4;
            }
            return 0.4;
        } else {
            if (targetPosition < currentPosition) {
                return -1;
            }
            return 1;
        }
    }

    public void update(double joystick) {
        if (manual) {
            setPower(joystick); // Maybe negative, depends on the motor
        } else {
            setPower(calculate(lift.getCurrentPosition()));
        }
    }
}
