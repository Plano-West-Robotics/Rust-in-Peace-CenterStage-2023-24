package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeDifferential;

@Autonomous(name="Outtake Arm Tester RIGHT")
public class OuttakeArmTesterRIGHT extends LinearOpMode {

    OuttakeDifferential arm;
    DroneLauncher drone;
    Intake intake;
    OuttakeBox box;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new OuttakeDifferential(hardwareMap, telemetry, OuttakeDifferential.State.DOWN);
        drone = new DroneLauncher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        box = new OuttakeBox(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Initialized", "True");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        arm.goTo(OuttakeDifferential.State.RIGHT);
        sleep(4000);
        arm.goTo(OuttakeDifferential.State.DOWN);
        sleep(4000);
    }
}