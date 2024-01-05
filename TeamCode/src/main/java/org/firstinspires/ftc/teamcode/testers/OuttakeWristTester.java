package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;

@Autonomous(name="Outtake Wrist Tester")
public class OuttakeWristTester extends LinearOpMode {

    OuttakeBox box;

    @Override
    public void runOpMode() throws InterruptedException {
        box = new OuttakeBox(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Initialized", "True");
            telemetry.addData("Position: ", box.getWristPosition());
            telemetry.update();
        }
        // run until the end of the match (driver presses STOP)
//

        box.wrist.setPosition(0); // scoring
        sleep(3000);
        box.wrist.setPosition(0.69); // down
        sleep(3000);
    }
}