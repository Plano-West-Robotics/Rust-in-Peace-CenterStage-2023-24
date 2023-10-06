package org.firstinspires.ftc.teamcode.subsystems.extra;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DCMotorArm {
    private final DcMotor arm;
    private final Telemetry telemetry;
    public DCMotorArm(HardwareMap hardwareMap, Telemetry telemetry){
        arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.telemetry = telemetry;
    }

    public void setPosition(int position) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
