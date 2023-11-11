package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetectionSub {

    private final AprilTagProcessor aprilTagProcessor;
    private final Telemetry telemetry;

    public enum Alliance {
        BLUE,
        RED
    }

    public AprilTagDetectionSub(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    }

    public ArrayList<AprilTagDetection> detectTags() {
        return aprilTagProcessor.getDetections();
    }

    private AprilTagDetection getTagById(List<AprilTagDetection> detections, int id) {
        for (AprilTagDetection detection : detections) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getLeftTag(Alliance color) {
        int targetId = (color == Alliance.BLUE) ? 1 : 4;
        return getTagById(detectTags(), targetId);
    }

    public AprilTagDetection getCenterTag(Alliance color) {
        int targetId = (color == Alliance.BLUE) ? 2 : 5;
        return getTagById(detectTags(), targetId);
    }

    public AprilTagDetection getRightTag(Alliance color) {
        int targetId = (color == Alliance.BLUE) ? 3 : 6;
        return getTagById(detectTags(), targetId);
    }
}
