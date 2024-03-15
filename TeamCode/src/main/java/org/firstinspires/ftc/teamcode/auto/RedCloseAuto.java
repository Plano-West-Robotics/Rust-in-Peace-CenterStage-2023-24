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
import org.firstinspires.ftc.teamcode.auto.roadrunner.trajectorysequence.TrajectorySequence;
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

        TrajectorySequence toSpikeMarkLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -31, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(4,-32))
                .build();

        Trajectory toBackdropLeft = drive.trajectoryBuilder(toSpikeMarkLeft.end())
                .splineToLinearHeading(new Pose2d(49, -31, Math.toRadians(0)), Math.toRadians(20))
                .build();

        TrajectorySequence toCyclePosition = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                .lineToConstantHeading(new Vector2d(20,-12))
                .build();

        TrajectorySequence toStackLeft = drive.trajectorySequenceBuilder(toCyclePosition.end())
                .lineToConstantHeading(new Vector2d(-65, -13),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence toBackDropLeftCycle = drive.trajectorySequenceBuilder(toStackLeft.end())
                .lineToConstantHeading(new Vector2d(30, -12),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(48, -37),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//
//        Trajectory toParkLeft = drive.trajectoryBuilder(toBackdropLeft.end(), true)
//                .splineToLinearHeading(new Pose2d(48, -57, Math.toRadians(90)), Math.toRadians(0))
//                .build();

        // CENTER ------------------- //
        Trajectory toSpikeMarkCenter = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(8, -30), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropCenter = drive.trajectoryBuilder(toSpikeMarkCenter.end())
                .splineToLinearHeading(new Pose2d(42, -31, Math.toRadians(0)), Math.toRadians(0))
                .build();

//        Trajectory toParkCenter = drive.trajectoryBuilder(toBackdropCenter.end(), true)
//                .splineToLinearHeading(new Pose2d(48, -57, Math.toRadians(90)), Math.toRadians(0))
//                .build();

        // RIGHT ------------------- //

        Trajectory toSpikeMarkRight = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(19, -38), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toBackdropRight = drive.trajectoryBuilder(toSpikeMarkRight.end())
                .splineToLinearHeading(new Pose2d(46, -34, Math.toRadians(0)), Math.toRadians(0))
                .build();

//        Trajectory toParkRight = drive.trajectoryBuilder(toBackdropRight.end(), true)
//                .splineToLinearHeading(new Pose2d(48, -57, Math.toRadians(90)), Math.toRadians(0))
//                .build();

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
        sleep(500);

        if (location == PropDetectionProcessor.Location.Left) {
            // Location.Left
            drive.setPoseEstimate(startPose);
            drive.followTrajectorySequence(toSpikeMarkLeft);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(450);

            // drop yellow pixel
            new Thread(() -> {
                outtake.goTo(OuttakeDifferential.State.LEFT);
                while(lift.getEncoderValue() < 110 && opModeIsActive()) {
                    lift.setPower(0.5);
                }
                lift.setPower(0.1);
            }).start();
            drive.followTrajectory(toBackdropLeft);

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            long curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (outtake.boxIsEmpty()) break;
            }
            sleep(500);
            drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
            sleep(500);
            new Thread(() -> {
                outtake.goTo(OuttakeDifferential.State.DOWN);
                while(lift.getEncoderValue() > 10 && opModeIsActive()) {
                    lift.setPower(-0.5);
                }
                lift.setPower(0);
            }).start();

           drive.followTrajectorySequence(toCyclePosition);
            intake.setTargetPositionPreset(Intake.Position.P5);
            intake.update();
            intake.spinForward();
            outtake.box.intake();
           drive.followTrajectorySequence(toStackLeft);
           drive.setMotorPowers(-0.1,-0.1,-0.1,-0.1);
           sleep(400);
            intake.setTargetPositionPreset(Intake.Position.P4);
            intake.update();
           sleep(700);
           drive.setMotorPowers(0,0,0,0);
            intake.stopSpin();

            drive.followTrajectorySequence(toBackDropLeftCycle);
            outtake.box.stopSpinning();
            new Thread(() -> {
                outtake.goTo(OuttakeDifferential.State.LEFT);
                while(lift.getEncoderValue() < 210 && opModeIsActive()) {
                    lift.setPower(0.5);
                }
                lift.setPower(0.1);
            }).start();
            drive.setMotorPowers(0.25, 0.25, 0.25, 0.25);
            sleep(1000);
            drive.setMotorPowers(0, 0, 0, 0);
            new Thread(() -> {
                while(lift.getEncoderValue() < 410 && opModeIsActive()) {
                    lift.setPower(0.5);
                }
                lift.setPower(0.1);
            }).start();
            sleep(500);
            new Thread(() -> {
                sleep(500);
                while(lift.getEncoderValue() > 10 && opModeIsActive()) {
                    lift.setPower(-0.5);
                }
                lift.setPower(0);
                outtake.setWrist(OuttakeDifferential.WristState.PASSIVE);

                outtake.goTo(OuttakeDifferential.State.DOWN);
                sleep(2000);
            }).start();
            drive.turn(Math.toRadians(90));

        } else if (location == PropDetectionProcessor.Location.Center) {
            // Location.Center
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkCenter);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(750);

            // drop yellow pixel
            outtake.goTo((OuttakeDifferential.State.UP));
            drive.followTrajectory(toBackdropRight);

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            long curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (outtake.boxIsEmpty()) break;
            }
            sleep(1500);
            drive.setMotorPowers(0, 0, 0, 0);
//            drive.followTrajectory(toParkCenter);
            outtake.goTo(OuttakeDifferential.State.DOWN);
            sleep(4000);
        } else {
            // Location.Right
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(toSpikeMarkRight);

            // drop purple pixel
            intake.setTargetPositionPreset(Intake.Position.TOP);
            intake.update();
            sleep(750);

            // drop yellow pixel
            new Thread(() -> {
                outtake.goTo(OuttakeDifferential.State.RIGHT);
                while(lift.getEncoderValue() < 110 && opModeIsActive()) {
                    lift.setPower(0.5);
                }
                lift.setPower(0.1);
            }).start();

            drive.followTrajectory(toBackdropRight);

            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            long curr = System.currentTimeMillis();
            while (System.currentTimeMillis() - curr < 2500 && opModeIsActive()) {
                if (outtake.boxIsEmpty()) break;
            }
            sleep(1500);
            drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
            sleep(500);
            new Thread(() -> {
                outtake.goTo(OuttakeDifferential.State.DOWN);
                while(lift.getEncoderValue() > 10 && opModeIsActive()) {
                    lift.setPower(-0.5);
                }
                lift.setPower(0);
            }).start();
//            drive.followTrajectory(toParkRight);
            sleep(4000);
        }

        Data.liftPosition = lift.getEncoderValue();
        visionPortal.close();
    }
}
