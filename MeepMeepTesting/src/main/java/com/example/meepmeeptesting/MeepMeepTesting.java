package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(80), Math.toRadians(80), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                                .back(5)
                                .turn(Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    // lift slide to prep for scoring
//                                    lift.setPower(1, true);
//                                    control.armDown();
                                })
                                .addTemporalMarker(0.5, () -> {
                                    // stall slide, arm up
//                                    lift.setPower(0.1, true);
//                                    control.armUp();
                                })
                                .splineToConstantHeading(new Vector2d(48, -42), 0)

                                .splineToConstantHeading(new Vector2d(33, -32), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(46, -52, Math.toRadians(90)), Math.toRadians(0))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}