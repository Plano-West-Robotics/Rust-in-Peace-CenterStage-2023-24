package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Data;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeDifferential;
import org.firstinspires.ftc.teamcode.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Red Backdrop-Side Auto", group="Auto")
public class RedCloseAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private PropDetectionProcessor propDetector;
    private VisionPortal visionPortal;

    Lift lift;
    Intake intake;
    OuttakeDifferential outtake;

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
        outtake = new OuttakeDifferential(hardwareMap, telemetry, OuttakeDifferential.State.DOWN);

        lift.setManual(false);

        // ----- TRAJECTORIES ----- //

        Pose2d startPose = new Pose2d(12, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // LEFT ------------------- //

        Trajectory toSpikeMarkLeft = drive.trajectoryBuilder(startPose, true)
                .splineToLinearHeading(new Pose2d(1, -38, Math.toRadians(320)), Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropLeft = drive.trajectoryBuilder(toSpikeMarkLeft.end())
                .splineToLinearHeading(new Pose2d(52, -32, Math.toRadians(0)), Math.toRadians(20))
                .build();

        Trajectory toParkLeft = drive.trajectoryBuilder(toBackdropLeft.end(), true)
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)), Math.toRadians(0))
                .build();

        // CENTER ------------------- //
        Trajectory toSpikeMarkCenter = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(8, -32), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropCenter = drive.trajectoryBuilder(toSpikeMarkCenter.end())
                .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory toParkCenter = drive.trajectoryBuilder(toBackdropCenter.end(), true)
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)), Math.toRadians(0))
                .build();

        // RIGHT ------------------- //

        Trajectory toSpikeMarkRight = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(17, -38), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropRight = drive.trajectoryBuilder(toSpikeMarkRight.end())
                .splineToLinearHeading(new Pose2d(52, -41, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory toParkRight = drive.trajectoryBuilder(toBackdropRight.end(), true)
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)), Math.toRadians(0))
                .build();

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
            // Location.Left
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkLeft);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(1500);

            // drop yellow pixel
            outtake.goTo(OuttakeDifferential.State.UP, true);
            drive.followTrajectory(toBackdropLeft);
            outtake.goTo(OuttakeDifferential.State.DOWN, true);

            drive.followTrajectory(toParkLeft);
        } else if (location == PropDetectionProcessor.Location.Center) {
            // Location.Center
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkCenter);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(1500);

            // drop yellow pixel
            outtake.goTo(OuttakeDifferential.State.UP, true);
            drive.followTrajectory(toBackdropCenter);
            outtake.goTo(OuttakeDifferential.State.DOWN, true);

            drive.followTrajectory(toParkCenter);
        } else {
            // Location.Right
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkRight);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(1500);

            // drop yellow pixel
            outtake.goTo(OuttakeDifferential.State.UP, true);
            drive.followTrajectory(toBackdropRight);
            outtake.goTo(OuttakeDifferential.State.DOWN, true);

            drive.followTrajectory(toParkRight);
        }

        Data.liftPosition = lift.getEncoderValue();
        visionPortal.close();
    }
}
