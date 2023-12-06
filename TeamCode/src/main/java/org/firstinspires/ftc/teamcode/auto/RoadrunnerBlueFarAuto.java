package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Data;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeController;
import org.firstinspires.ftc.teamcode.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Roadrunner Blue Audience-Side Auto", group="Auto")
public class RoadrunnerBlueFarAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private PropDetectionProcessor propDetector;
    private VisionPortal visionPortal;

    Lift lift;
    Intake intake;
    OuttakeController control;

    PropDetectionProcessor.Location location;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----- HARDWARE INITIALIZATION ---- //

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        propDetector = new PropDetectionProcessor();
        propDetector.propColor = PropDetectionProcessor.Prop.BLUE;
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "CAM"), propDetector);

        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        control = new OuttakeController(hardwareMap, telemetry, lift);

        lift.setManual(false);

        // ----- TRAJECTORIES ----- //

        Pose2d startPose = new Pose2d(-36, 60, 0);
        drive.setPoseEstimate(startPose);

        List<Trajectory> allTrajectories = getTrajectories(drive, startPose);
        List<Trajectory> path = new ArrayList<>();

        // ----- WAIT FOR START ----- //

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine(String.valueOf(propDetector.getLocation()));
            telemetry.update();

            location = propDetector.getLocation();
        }

        // ----- START PRESSED ----- //

        runtime.reset();

        if(isStopRequested()) return;

        // ----- RUNNING OP-MODE ----- //

        if (location == PropDetectionProcessor.Location.Left) {
            path.add(allTrajectories.get(0));
            path.add(allTrajectories.get(1));
            path.add(allTrajectories.get(2));
        } else if (location == PropDetectionProcessor.Location.Center) {
            path.add(allTrajectories.get(3));
            path.add(allTrajectories.get(4));
            path.add(allTrajectories.get(5));
        } else {
            path.add(allTrajectories.get(6));
            path.add(allTrajectories.get(7));
            path.add(allTrajectories.get(8));
        }

        // prepare for movement (set arm to down, lift slide)
        control.armDown();
        lift.setPower(0.7, true);
        sleep(500);
        lift.setPower(0.1, true);

        drive.followTrajectory(path.get(0));

        // drop purple pixel
        intake.setSpeed(0.2);
        intake.spinForward();
        sleep(500);
        intake.stopSpin();

        drive.followTrajectory(path.get(1));

        // drop yellow pixel
        control.spinBoxOut();
        sleep(3000);
        control.stopBox();

        drive.followTrajectory(path.get(2));
        drive.turn(Math.toRadians(-90));

        Data.liftPosition = lift.getEncoderValue();
        visionPortal.close();
    }

    private List<Trajectory> getTrajectories(SampleMecanumDrive drive, Pose2d startPose) {
        List<Trajectory> path = new ArrayList<>();

        Trajectory trajectoryToSpikeMark, trajectoryToBackdrop, trajectoryToPark;

        // Location.Left
        trajectoryToSpikeMark = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-40, 48, Math.toRadians(180)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-36, 32), Math.toRadians(270))
                .build();

        trajectoryToBackdrop = drive.trajectoryBuilder(trajectoryToSpikeMark.end())
                .splineToConstantHeading(new Vector2d(-48, 16), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32, 8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(16, 8, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // lift slide to prep for scoring
                    lift.setPower(0.5, true);
                })
                .splineToConstantHeading(new Vector2d(38, 14), Math.toRadians(0))
                .addTemporalMarker(6, () -> {
                    // stall slide, arm up
                    lift.setPower(0.1, true);
                    control.armUp();
                })
                .splineToConstantHeading(new Vector2d(48, 42), Math.toRadians(0))
                .build();

        trajectoryToPark = drive.trajectoryBuilder(trajectoryToBackdrop.end())
                .strafeRight(30)
                .addDisplacementMarker(() -> {
                    control.armDown();
                })
                .build();

        path.add(trajectoryToSpikeMark);
        path.add(trajectoryToBackdrop);
        path.add(trajectoryToPark);

        // Location.Center
        trajectoryToSpikeMark = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-44, 40), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(270)), Math.toRadians(0))
                .build();

        trajectoryToBackdrop = drive.trajectoryBuilder(trajectoryToSpikeMark.end())
                .splineToSplineHeading(new Pose2d(-12, 6, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // lift slide to prep for scoring
                    lift.setPower(0.5, true);
                })
                .splineToConstantHeading(new Vector2d(36, 18), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // stall slide, arm up
                    lift.setPower(0.1, true);
                    control.armUp();
                })
                .splineToConstantHeading(new Vector2d(48, 36), Math.toRadians(0))
                .build();

        trajectoryToPark = drive.trajectoryBuilder(trajectoryToBackdrop.end())
                .strafeRight(24)
                .addDisplacementMarker(() -> {
                    control.armDown();
                })
                .build();

        path.add(trajectoryToSpikeMark);
        path.add(trajectoryToBackdrop);
        path.add(trajectoryToPark);

        // Location.Right
        trajectoryToSpikeMark = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-40, 54), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-34, 34), Math.toRadians(270))
                .build();

        trajectoryToBackdrop = drive.trajectoryBuilder(trajectoryToSpikeMark.end())
                .splineToConstantHeading(new Vector2d(-34, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // lift slide to prep for scoring
                    lift.setPower(0.5, true);
                })
                .splineToConstantHeading(new Vector2d(36, 24), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // stall slide, arm up
                    lift.setPower(0.1, true);
                    control.armUp();
                })
                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(0))
                .build();

        trajectoryToPark = drive.trajectoryBuilder(trajectoryToBackdrop.end())
                .strafeRight(18)
                .addDisplacementMarker(() -> {
                    control.armDown();
                })
                .build();

        path.add(trajectoryToSpikeMark);
        path.add(trajectoryToBackdrop);
        path.add(trajectoryToPark);

        return path;
    }
}
