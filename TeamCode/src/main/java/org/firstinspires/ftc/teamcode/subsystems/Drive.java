package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class Drive {

    // Null by default
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private double speed;

    private final Telemetry telemetry;
    private final Imu imu;

    private final int TICKS_PER_DEGREES = 11;

    // This is a constructor
    public Drive(HardwareMap hardwareMap, Telemetry telemetry, boolean auto) {

        // Instantiates motors
        frontLeft = hardwareMap.get(DcMotor.class,"FLmotor");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");

        // Sets direction for motors -- forward is default
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motor to run by RPM and not voltage
        List<DcMotor> motors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);

        for (DcMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (auto) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        speed = 1;

        imu = new Imu(hardwareMap, telemetry);

        // Assigns telemetry
        this.telemetry = telemetry;
    }

    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
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

    /**
     * Updates the power of all four motors of the Drive object
     * depending on analog x, y, and rx values.
     * @param x the translational X direction
     * @param y the translational Y direction
     * @param rx the rotation coefficient (heading)
     */
    public void update(double x, double y, double rx) {
        x *= 1.1; // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double brPower = (y + x - rx) / denominator;
        setDrivePowers(flPower, frPower, blPower, brPower);
    }

    /**
     * Updates the power of all four motors of the Drive object
     * depending on analog x, y, and rx values to create a field-centric drive.
     * @param x the translational X direction
     * @param y the translational Y direction
     * @param rx the rotation coefficient (heading)
     */
    public void updateFieldCentric(double x, double y, double rx) {
        double botHeading = -imu.getHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY + rotX - rx) / denominator;
        double blPower = (rotY + rotX + rx) / denominator;
        double brPower = (rotY - rotX - rx) / denominator;
        setDrivePowers(flPower, frPower, blPower, brPower);
    }

    public void resetHeading() {
        imu.reset();
    }

    public int[] getEncoderValues() {
        return new int[]{frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition()};
    }

    // TeleOp
    public void moveForward(int ticks) {
        frontLeft.setTargetPosition(calculateTicks(frontLeft, ticks));
        frontRight.setTargetPosition(calculateTicks(frontRight, ticks));
        backLeft.setTargetPosition(calculateTicks(backLeft, ticks));
        backRight.setTargetPosition(calculateTicks(backRight, ticks));

        changeMotorModeToPosition();
        setDrivePowers(speed,speed,speed,speed);
        block();
    }

    public void moveBackward(int ticks) {
        frontLeft.setTargetPosition(calculateTicks(frontLeft, -ticks));
        frontRight.setTargetPosition(calculateTicks(frontRight, -ticks));
        backLeft.setTargetPosition(calculateTicks(backLeft, -ticks));
        backRight.setTargetPosition(calculateTicks(backRight, -ticks));

        changeMotorModeToPosition();
        setDrivePowers(speed,speed,speed,speed);
        block();
    }

    public void strafeLeft(int ticks) {
        frontLeft.setTargetPosition(calculateTicks(frontLeft, -ticks));
        frontRight.setTargetPosition(calculateTicks(frontRight, ticks));
        backLeft.setTargetPosition(calculateTicks(backLeft, ticks));
        backRight.setTargetPosition(calculateTicks(backRight, -ticks));

        changeMotorModeToPosition();
        setDrivePowers(speed,speed,speed,speed);
        block();
    }

    public void strafeRight(int ticks) {
        frontLeft.setTargetPosition(calculateTicks(frontLeft, ticks));
        frontRight.setTargetPosition(calculateTicks(frontRight, -ticks));
        backLeft.setTargetPosition(calculateTicks(backLeft, -ticks));
        backRight.setTargetPosition(calculateTicks(backRight, ticks));

        changeMotorModeToPosition();
        setDrivePowers(speed,speed,speed,speed);
        block();
    }

    public void turnLeft(int degrees) {
        int ticks = degrees * TICKS_PER_DEGREES;

        frontLeft.setTargetPosition(calculateTicks(frontLeft, -ticks));
        frontRight.setTargetPosition(calculateTicks(frontRight, ticks));
        backLeft.setTargetPosition(calculateTicks(backLeft, -ticks));
        backRight.setTargetPosition(calculateTicks(backRight, ticks));

        changeMotorModeToPosition();
        setDrivePowers(speed,speed,speed,speed);
        block();
    }

    public void turnRight(int degrees) {
        int ticks = degrees * TICKS_PER_DEGREES;

        frontLeft.setTargetPosition(calculateTicks(frontLeft, ticks));
        frontRight.setTargetPosition(calculateTicks(frontRight, -ticks));
        backLeft.setTargetPosition(calculateTicks(backLeft, ticks));
        backRight.setTargetPosition(calculateTicks(backRight, -ticks));

        changeMotorModeToPosition();
        setDrivePowers(speed,speed,speed,speed);
        block();
    }

    public int calculateTicks(DcMotor motor, int distanceTicks) {
        return distanceTicks + motor.getCurrentPosition();
    }

    public void changeMotorModeToPosition(){
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void block(){
        while ((frontLeft.isBusy() || frontRight.isBusy()) || (backLeft.isBusy() || backRight.isBusy())) {
            // take up resources
        }
    }

    public double getHeading() {
        return imu.getHeading();
    }
}