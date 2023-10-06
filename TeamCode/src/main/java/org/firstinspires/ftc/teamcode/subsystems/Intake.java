package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private final Telemetry telemetry;
    private final Servo arm;
    private final DcMotor intake;
    private double speed;
    private int targetPosition;
    enum Position {
        TOP,
        MIDDLE,
        DOWN
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        arm = hardwareMap.get(Servo.class,"arm");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        speed = 1;
    }
    public void setTargetPositionPreset(Position targetPosition) {
        switch (targetPosition) {
            case TOP: setTargetPosition(100); break;
            case MIDDLE: setTargetPosition(50); break;
            case DOWN: setTargetPosition(0); break;
        }
    }
    public void setTargetPosition(int targetPosition) {
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
}