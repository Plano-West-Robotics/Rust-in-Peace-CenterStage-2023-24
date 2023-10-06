package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AryanClaw {
    private final Servo claw;
    private final Telemetry telemetry;
    public static final double POS_OPEN = 0;
    public static final double POS_CLOSE = 1;

    // Constants to set servo position to open or close
    public AryanClaw(HardwareMap hardwareMap, Telemetry telemetry){
        claw = hardwareMap.get(Servo.class,"claw");
        this.telemetry = telemetry;
    }
    public void goTo(double position) {
        claw.setPosition(position);
    }
    public void closeClaw(){
        claw.setPosition(POS_CLOSE);
    }
    public void openClaw(){
        claw.setPosition(POS_OPEN);
    }
}
