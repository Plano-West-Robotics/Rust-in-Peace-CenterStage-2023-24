package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private final Telemetry telemetry;
    public final Servo arm;
    private final DcMotor intake;
    private double speed;
    private double targetPosition;
    public enum Position {
        TOP,
        P5,
        P4,
        P3,
        P2,
        DOWN
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        arm = hardwareMap.get(Servo.class,"ARMservo");
        arm.scaleRange(0.57, 1);
        arm.setDirection(Servo.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "INmotor");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        speed = 0.7;

        targetPosition = intake.getCurrentPosition();
    }
    public void setTargetPositionPreset(Position targetPosition) {
        switch (targetPosition) {
            case TOP: setTargetPosition(1); break;
            case P5: setTargetPosition(0.54); break; // 0.57
            case P4: setTargetPosition(0.45); break;
            case P3: setTargetPosition(0.35); break;
            case P2: setTargetPosition(0.25); break;
            case DOWN: setTargetPosition(0); break;
        }
    }
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void spinForward(){
        intake.setPower(1 * speed);
    }
    public void spinBackwards(){
        intake.setPower(-1 * speed);
    }
    public void stopSpin(){
        intake.setPower(0);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public double getSpeed() {
        return speed;
    }

    public double getCurrentPosition() {
        return arm.getPosition();
    }

    public void update() {
        arm.setPosition(targetPosition);
    }
}