package org.firstinspires.ftc.teamcode.testers;

import android.bluetooth.BluetoothClass;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp(name = "MotorTesterInator")
public class MotorTesterInator extends OpMode {
    private double speed;
    DcMotorImplEx motorEx;
    int numMotors;
    @Override
    public void init() {
        //motorEx = new DcMotorImplEx();//(DcMotorImplEx)motor;
        HardwareMap.DeviceMapping<DcMotor> hw = hardwareMap.new DeviceMapping<>(DcMotor.class);
        numMotors = hw.size();
    }

    @Override
    public void loop() {
        //motorEx.setPower(gamepad1.right_trigger + gamepad1.left_trigger);

        telemetry.addData("number of motors:", numMotors);
    }
}
