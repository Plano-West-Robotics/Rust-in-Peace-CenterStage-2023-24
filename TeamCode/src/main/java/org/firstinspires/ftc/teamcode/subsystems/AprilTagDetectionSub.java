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
        if (detections.size() > 0) {
            return detections.get(0);
        } else {
            // Handle the case when no left tag is detected
            return null;
        }
    }

    public AprilTagDetection getCenterTag() {
        ArrayList<AprilTagDetection> detections = detectTags();
        if (detections.size() > 1) {
            return detections.get(1);
        } else {
            // Handle the case when no center tag is detected
            return null;
        }
    }

    public AprilTagDetection getRightTag() {
        ArrayList<AprilTagDetection> detections = detectTags();
        if (detections.size() > 2) {
            return detections.get(2);
        } else {
            // Handle the case when no right tag is detected
            return null;
        }
    }
}
