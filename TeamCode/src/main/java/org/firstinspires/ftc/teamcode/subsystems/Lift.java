package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private final Telemetry telemetry;

    private final DcMotor slideLeft;
    private final DcMotor slideRight;

    private boolean manual;
    private double speed;
    private int targetPosition;

    private int liftOffset;

    public enum Position {
        BOTTOM,
        MEDIUM,
        HIGH
    }
    // THIS IS A CONSTRUCTOR

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        slideLeft = hardwareMap.get(DcMotor.class, "SLmotor");
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideRight = hardwareMap.get(DcMotor.class, "SRmotor");
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideRight.setDirection(DcMotor.Direction.REVERSE);

        manual = true;
        // a value between 0 and 1 where 1 is max speed
        speed = 1;

        liftOffset = slideRight.getCurrentPosition();

        setTargetPositionPreset(Position.BOTTOM);

        this.telemetry = telemetry;
    }

    //region Lift
    public void setManual(boolean manual) {
        this.manual = manual;
    }

    public void setPower(double power, boolean override) {
        if (!override) {
            int height = getEncoderValue();
            if (power == 0) {
                slideLeft.setPower(power * speed);
                slideRight.setPower(power * speed);
            } else if (power < 0) {
                if (height < 45) power = 0;
                slideLeft.setPower(power * speed);
                slideRight.setPower(power * speed);
            } else if (power > 0) {
                if (height > 2100) power = 0;
                slideLeft.setPower(power * speed);
                slideRight.setPower(power * speed);
            }
        } else {
            slideLeft.setPower(power * speed);
            slideRight.setPower(power * speed);
        }
    }

    public void setPower(double power) {
        int height = getEncoderValue();
        if (power == 0) {
            slideLeft.setPower(power * speed);
            slideRight.setPower(power * speed);
        } else if (power < 0) {
            if (height < 45) power = 0.1;
            slideLeft.setPower(power * speed);
            slideRight.setPower(power * speed);
        } else if (power > 0) {
            slideLeft.setPower(power * speed);
            slideRight.setPower(power * speed);
        }
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setTargetPositionPreset(Position targetPosition) {
        switch (targetPosition) {
            case BOTTOM: setTargetPosition(45); break; // Generally values from 0 to 1000
            case MEDIUM: setTargetPosition(1000); break;
            case HIGH: setTargetPosition(2000); break;
        }
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    /** calculates the power that the slide needs
     *
     * @param currentPosition
     * @return
     */
    public double calculate(int currentPosition) {

        int distance = Math.abs(targetPosition - currentPosition);
        if (distance < 20) {
            manual = true;
            return 0.1;
        } else if (distance < 100) {
            if (targetPosition < currentPosition) {
                return -0.2;
            }
            return 0.4;
        } else {
            if (targetPosition < currentPosition) {
                return -0.5;
            }
            return 1;
        }
    }

    public void update(double joystick) {
        if (manual) {
            setPower(joystick);
        } else {
            setPower(calculate(getEncoderValue()));
        }
        Data.liftPosition = getEncoderValue();
    }

    public int getEncoderValue() {
        return slideRight.getCurrentPosition()-liftOffset;
    }
    public void resetEncoder() {
        liftOffset = slideRight.getCurrentPosition();
    }

    public boolean getManual() {
        return manual;
    }

    public void loadPosition() {
        liftOffset -= Data.liftPosition;
    }

    //endregion
}
