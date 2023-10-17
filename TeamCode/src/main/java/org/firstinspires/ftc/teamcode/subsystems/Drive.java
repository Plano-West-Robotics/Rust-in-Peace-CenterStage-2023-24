package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {

    // Null by default
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private double speed;
    private final Telemetry telemetry;

    // This is a constructor
    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {

        // Instantiates motors
        frontLeft = hardwareMap.get(DcMotor.class,"FLmotor");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");

        // Sets direction for motors -- forward is default
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set mode is good practice -- RUN_WITHOUT_ENCODER is default
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        speed = 1;

        // Assigns telemetry
        this.telemetry = telemetry;
    }

    private void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        frontLeft.setPower(flPower * speed);
        frontRight.setPower(frPower * speed);
        backLeft.setPower(blPower * speed);
        backRight.setPower(brPower * speed);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public void updateBackwards(double x, double y, double rx) {
        update(x, -y, rx);
    }

    public void update(double x, double y, double rx) {
        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;
        setDrivePowers(flPower, frPower, blPower, brPower);
    }
}