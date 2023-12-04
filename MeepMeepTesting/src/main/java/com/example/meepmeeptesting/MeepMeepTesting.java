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
                // TODO: Change MeepMeep constraints after tuning RR
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, 0))
                                .addDisplacementMarker(() -> {
                                    // prepare for movement
                                })

                                // ----- trajectoryToSpikeMark -----
                                .splineToConstantHeading(new Vector2d(-40, 54), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-34, 34), Math.toRadians(270))
                                // ----- END -----

                                .addDisplacementMarker(() -> {
                                    // place purple pixel!
                                })

                                // ----- trajectoryToBackdrop -----
                                .splineToConstantHeading(new Vector2d(-34, 12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    // lift slide to prep for scoring
//                                    lift.setPower(0.5, true);
                                })
                                .splineToConstantHeading(new Vector2d(36, 24), Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    // stall slide, arm up
//                                    lift.setPower(0.1, true);
//                                    control.armUp();
                                })
                                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(0))
                                // ----- END -----

                                .addDisplacementMarker(() -> {
                                    // drop yellow pixel!
                                })

                                // ----- trajectoryToPark -----
                                .strafeRight(18)
                                .addDisplacementMarker(() -> {
//                                    control.armDown();
                                })
                                .turn(Math.toRadians(-90))
                                // ----- END -----

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}