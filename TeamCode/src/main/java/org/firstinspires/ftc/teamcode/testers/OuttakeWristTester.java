package org.firstinspires.ftc.teamcode.testers;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;

@Disabled
@Autonomous(name="Outtake Wrist Tester")
public class OuttakeWristTester extends LinearOpMode {

    OuttakeBox box;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        box = new OuttakeBox(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Initialized", "True");
            telemetry.addData("Position: ", box.getWristPosition());
            telemetry.update();
        }
        // run until the end of the match (driver presses STOP)

        while (lift.getEncoderValue() < 500 && opModeIsActive()) {
            lift.setPower(0.7);
        }
        lift.setPower(0.1);
        box.setWristPosition(OuttakeBox.State.P4);
        sleep(3000);
        box.setWristPosition(OuttakeBox.State.P3);
        sleep(3000);
    }
}