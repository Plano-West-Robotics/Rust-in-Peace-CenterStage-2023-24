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

@Autonomous(name="Blue Backdrop-Side Auto", group="Auto")
public class BlueCloseAuto extends LinearOpMode {

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
        propDetector.propColor = PropDetectionProcessor.Prop.BLUE;
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "CAM"), propDetector);

        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new OuttakeDifferential(hardwareMap, telemetry, OuttakeDifferential.State.DOWN);

        lift.setManual(false);

        // ----- TRAJECTORIES ----- //

        Pose2d startPose = new Pose2d(13, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // LEFT ------------------- //

        Trajectory toSpikeMarkLeft = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(28, 41), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropLeft = drive.trajectoryBuilder(toSpikeMarkLeft.end())
                .splineToLinearHeading(new Pose2d(40, 37.5, Math.toRadians(0)), Math.toRadians(-20))
                .build();

        Trajectory toParkLeft = drive.trajectoryBuilder(toBackdropLeft.end(), true)
                .splineToLinearHeading(new Pose2d(50, 60, Math.toRadians(270)), Math.toRadians(0))
                .build();

        // CENTER ------------------- //
        Trajectory toSpikeMarkCenter = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(20, 34), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropCenter = drive.trajectoryBuilder(toSpikeMarkCenter.end())
                .splineToLinearHeading(new Pose2d(47, 33.5, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory toParkCenter = drive.trajectoryBuilder(toBackdropCenter.end(), true)
                .splineToLinearHeading(new Pose2d(50, 60, Math.toRadians(270)), Math.toRadians(0))
                .build();

        // RIGHT ------------------- //

        Trajectory toSpikeMarkRight = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(15.5, 48), Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(9, 36, Math.toRadians(50)), Math.toRadians(200),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropRight = drive.trajectoryBuilder(toSpikeMarkRight.end())
                .splineToLinearHeading(new Pose2d(40, 38, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory toParkRight = drive.trajectoryBuilder(toBackdropRight.end(), true)
                .splineToLinearHeading(new Pose2d(50, 60, Math.toRadians(270)), Math.toRadians(0))
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
        intake.setTargetPositionPreset(Intake.Position.DOWN);
        intake.update();
        sleep(1500);

        if (location == PropDetectionProcessor.Location.Left) {
            // Location.Left
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkLeft);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(1500);

            // drop yellow pixel
            outtake.goTo(OuttakeDifferential.State.UP);
            drive.followTrajectory(toBackdropLeft);
            outtake.setWrist(OuttakeDifferential.WristState.MANUAL);

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            long curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (System.currentTimeMillis() - curr > 2000) outtake.box.outtake();
            }
            sleep(1000);
            drive.setMotorPowers(0, 0, 0, 0);
            outtake.box.stopSpinning();
            outtake.setWrist(OuttakeDifferential.WristState.PASSIVE);


            outtake.goTo(OuttakeDifferential.State.DOWN);

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
            outtake.setWrist(OuttakeDifferential.WristState.MANUAL);

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            long curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (System.currentTimeMillis() - curr > 2000) outtake.box.outtake();
            }
            sleep(1000);
            drive.setMotorPowers(0, 0, 0, 0);
            outtake.box.stopSpinning();

            outtake.goTo(OuttakeDifferential.State.DOWN, true);

            drive.followTrajectory(toParkCenter);
        } else {
            // Location.Right
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkRight);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(750);

            // drop yellow pixel
            drive.followTrajectory(toBackdropRight);

            new Thread(() -> {
                outtake.goTo(OuttakeDifferential.State.FARRIGHT);
                outtake.setWrist(OuttakeDifferential.WristState.MANUAL);
                while(lift.getEncoderValue() < 400 && opModeIsActive()) {
                    lift.setPower(0.6);
                }
                lift.setPower(0.1);
            }).start();

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            long curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (System.currentTimeMillis() - curr > 2000) outtake.box.outtake();
            }
            sleep(1000);
            drive.setMotorPowers(0, 0, 0, 0);
            outtake.box.stopSpinning();

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (outtake.boxIsEmpty()) break;
            }
            sleep(1000);
            drive.setMotorPowers(0, 0, 0, 0);
            outtake.setWrist(OuttakeDifferential.WristState.PASSIVE);
            new Thread(() -> {
                sleep(200);
                while(lift.getEncoderValue() > 10 && opModeIsActive()) {
                    lift.setPower(-0.5);
                }
                lift.setPower(0);

                outtake.setWrist(OuttakeDifferential.WristState.PASSIVE);
                outtake.goTo(OuttakeDifferential.State.UP);
                sleep(2000);
            }).start();
            sleep(500);
            outtake.goTo(OuttakeDifferential.State.DOWN);
            drive.followTrajectory(toParkRight);
        }

        Data.liftPosition = lift.getEncoderValue();
        visionPortal.close();
    }
}
