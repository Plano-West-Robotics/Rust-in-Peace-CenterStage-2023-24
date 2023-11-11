package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagDetectionSub {

    private final AprilTagProcessor aprilTagProcessor;
    private final Telemetry telemetry;

    public AprilTagDetectionSub(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    }

    public ArrayList<AprilTagDetection> detectTags() {
        return aprilTagProcessor.getDetections();
    }

    public AprilTagDetection getLeftTag() {
        ArrayList<AprilTagDetection> detections = detectTags();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 1) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getCenterTag() {
        ArrayList<AprilTagDetection> detections = detectTags();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 2) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getRightTag() {
        ArrayList<AprilTagDetection> detections = detectTags();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 3) {
                return detection;
            }
        }
        return null;
    }
}
