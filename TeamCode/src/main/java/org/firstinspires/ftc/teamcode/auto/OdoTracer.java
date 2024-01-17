package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.auto.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.subsystems.Data;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeDifferential;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="OdoTracer", group="Auto")
public class OdoTracer extends OpMode {
    private TwoWheelTrackingLocalizer twoWheelOdo;
    private SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        twoWheelOdo = new TwoWheelTrackingLocalizer(hardwareMap, drive);
    }

    @Override
    public void loop() {
        telemetry.addData("Initialized", "True");
        telemetry.addData("Position: ", twoWheelOdo.getWheelPositions());
        telemetry.update();
    }

    //I'm only doing this cuz idk if the List toString prints out the array.
    //if it does, feel free to delete this function and change it on the telemetry.addline.
    private String listIterator(List<Double> ls){
        StringBuilder out = new StringBuilder("{");
        for(int i= 0; i < ls.size()-1; ++i){
            out.append(ls.get(i));
            out.append(", ");

        }
        out.append("}");

        return out.toString();
    }


    }
