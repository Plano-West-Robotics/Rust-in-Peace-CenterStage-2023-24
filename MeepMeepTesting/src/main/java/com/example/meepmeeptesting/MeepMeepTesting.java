package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    /**
     *
     Trajectory toBackdropCenter = drive.trajectoryBuilder(startPose)
     .splineToConstantHeading(new Vector2d(47, -39), Math.toRadians(0))
     .splineToConstantHeading(new Vector2d(50, -39), Math.toRadians(0),
     SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
     SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
     .build();

     Pose2d backdropEndCenter = new Pose2d(46, -39, 0);

     Trajectory toSpikeMarkCenter = drive.trajectoryBuilder(backdropEndCenter, true)
     .splineToConstantHeading(new Vector2d(14, -36), Math.toRadians(180))
     .build();

     Trajectory toParkCenter = drive.trajectoryBuilder(toSpikeMarkCenter.end())
     .splineToLinearHeading(new Pose2d(45, -60, Math.toRadians(90)), Math.toRadians(0))
     .build();

     * @param args
     */
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(100), Math.toRadians(100), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(16, 54, Math.toRadians(135)), Math.toRadians(315))
                                .splineToSplineHeading(new Pose2d(11, 41, Math.toRadians(50)), Math.toRadians(200))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}