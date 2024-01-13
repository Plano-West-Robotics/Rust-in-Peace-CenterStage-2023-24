package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeBox;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeDifferential;

@Autonomous(name="Outtake Arm Tester")
public class OuttakeArmTester extends LinearOpMode {

    OuttakeDifferential arm;
    DroneLauncher drone;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new OuttakeDifferential(hardwareMap, telemetry, OuttakeDifferential.State.DOWN);
        drone = new DroneLauncher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Initialized", "True");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        arm.goTo(OuttakeDifferential.State.UP);
        sleep(3000);
        arm.goTo(OuttakeDifferential.State.DOWN);
        sleep(3000);
    }
}