package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Imu;

@Autonomous(name="IMU Reset AUTO")
public class IMUresetor extends LinearOpMode {
    private Imu imu;

    public void runOpMode() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {
            imu.reset();
        }

        if(isStopRequested()) return;

        imu.reset();
    }
}
