package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name="Roadrunner Auto Test", group="Auto")
public class RoadrunnerAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory simpleTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        Trajectory splineTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(5, 0), 0)
                .splineTo(new Vector2d(5, 5), Math.toRadians(90))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(simpleTrajectory);
//        drive.followTrajectory(splineTrajectory);
    }
}
