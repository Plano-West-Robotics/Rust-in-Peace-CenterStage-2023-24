package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private final Telemetry telemetry;
    private final CRServo arm;
    private final DcMotor intake;
    private double speed;
    private double targetPosition;
    enum Position {
        TOP,
        MIDDLE,
        DOWN
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        arm = hardwareMap.get(CRServo.class,"ARMservo");
        arm.setDirection(CRServo.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "INmotor");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        speed = 1;
    }
    public void setTargetPositionPreset(Position targetPosition) {
        switch (targetPosition) {
            case TOP: setTargetPosition(1); break;
            case MIDDLE: setTargetPosition(0.5); break;
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

    public void setArmPower(double power) {
        arm.setPower(power);
    }
}