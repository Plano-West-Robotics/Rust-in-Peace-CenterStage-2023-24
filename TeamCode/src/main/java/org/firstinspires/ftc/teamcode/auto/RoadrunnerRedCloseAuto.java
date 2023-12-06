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

@Autonomous(name="Roadrunner Red Backdrop-Side Auto", group="Auto")
public class RoadrunnerRedCloseAuto extends LinearOpMode {

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
        propDetector.propColor = PropDetectionProcessor.Prop.RED;
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "CAM"), propDetector);

        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        control = new OuttakeController(hardwareMap, telemetry, lift);

        lift.setManual(false);

        // ----- TRAJECTORIES ----- //

        Pose2d startPose = new Pose2d(12, -60, 0);
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

//        if (location == PropDetectionProcessor.Location.Left) {
            path.add(allTrajectories.get(0));
            path.add(allTrajectories.get(1));
            path.add(allTrajectories.get(2));
//        } else if (location == PropDetectionProcessor.Location.Center) {
//            path.add(allTrajectories.get(3));
//            path.add(allTrajectories.get(4));
//            path.add(allTrajectories.get(5));
//        } else {
//            path.add(allTrajectories.get(6));
//            path.add(allTrajectories.get(7));
//            path.add(allTrajectories.get(8));
//        }

        // prepare for movement (set arm to down, lift slide)
        lift.setPower(0.7, true);
        sleep(500);
        lift.setPower(0.1, true);
        control.armDown();

        drive.followTrajectory(path.get(0));

        // drop yellow pixel
        control.spinBoxOut();
        sleep(3000);
        control.stopBox();

        drive.followTrajectory(path.get(1));

//        // drop purple pixel
//        intake.setSpeed(0.2);
//        intake.spinForward();
//        sleep(500);
//        intake.stopSpin();
//
//        drive.followTrajectory(path.get(2));

        Data.liftPosition = lift.getEncoderValue();
        visionPortal.close();
    }

    private List<Trajectory> getTrajectories(SampleMecanumDrive drive, Pose2d startPose) {
        List<Trajectory> path = new ArrayList<>();

        Trajectory trajectoryToBackdrop, trajectoryToSpikeMark, trajectoryToPark;

        // Location.Left
        trajectoryToBackdrop = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(20, -54), 0)
                .addDisplacementMarker(() -> {
                    // lift slide to prep for scoring
                    lift.setPower(0.9, true);
                })
                .addTemporalMarker(5, () -> {
                    // stall slide, arm up
                    lift.setPower(0.1, true);
                    control.armUp();
                })
                .splineToConstantHeading(new Vector2d(48, -30), 0)
                .build();

        trajectoryToSpikeMark = drive.trajectoryBuilder(trajectoryToBackdrop.end(), true)
                .addTemporalMarker(0.5, () -> {
                    // move arm down after leaving the board
                    control.armDown();
                })
                .splineTo(new Vector2d(15, -30), Math.toRadians(180))
                .build();

        trajectoryToPark = drive.trajectoryBuilder(trajectoryToSpikeMark.end(), true)
                .splineTo(new Vector2d(25, -40), Math.toRadians(0))
                .splineTo(new Vector2d(46, -52), Math.toRadians(270))
                .build();

        path.add(trajectoryToBackdrop);
        path.add(trajectoryToSpikeMark);
        path.add(trajectoryToPark);

        // Location.Center
        trajectoryToBackdrop = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(20, -54), 0)
                .addDisplacementMarker(() -> {
                    // lift slide to prep for scoring
                    lift.setPower(0.5, true);
                })
                .addTemporalMarker(2, () -> {
                    // stall slide, arm up
                    lift.setPower(0.1, true);
                    control.armUp();
                })
                .splineTo(new Vector2d(48, -36), 0)
                .build();

        trajectoryToSpikeMark = drive.trajectoryBuilder(trajectoryToBackdrop.end(), true)
                .addTemporalMarker(0.5, () -> {
                    // move arm down after leaving the board
                    control.armDown();
                })
                .splineTo(new Vector2d(30, -24), Math.toRadians(180))
                .build();

        trajectoryToPark = drive.trajectoryBuilder(trajectoryToSpikeMark.end(), true)
                .splineTo(new Vector2d(46, -52), Math.toRadians(270))
                .build();

        path.add(trajectoryToBackdrop);
        path.add(trajectoryToSpikeMark);
        path.add(trajectoryToPark);

        // Location.Right
        trajectoryToBackdrop = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(20, -54), 0)
                .addDisplacementMarker(() -> {
                    // lift slide to prep for scoring
                    lift.setPower(0.5, true);
                })
                .addTemporalMarker(2, () -> {
                    // stall slide, arm up
                    lift.setPower(0.1, true);
                    control.armUp();
                })
                .splineTo(new Vector2d(48, -42), 0)
                .build();

        trajectoryToSpikeMark = drive.trajectoryBuilder(trajectoryToBackdrop.end(), true)
                .addTemporalMarker(0.5, () -> {
                    // move arm down after leaving the board
                    control.armDown();
                })
                .splineTo(new Vector2d(34, -32), Math.toRadians(180))
                .build();

        trajectoryToPark = drive.trajectoryBuilder(trajectoryToSpikeMark.end(), true)
                .splineTo(new Vector2d(46, -52), Math.toRadians(270))
                .build();

        path.add(trajectoryToBackdrop);
        path.add(trajectoryToSpikeMark);
        path.add(trajectoryToPark);

        return path;
    }
}
