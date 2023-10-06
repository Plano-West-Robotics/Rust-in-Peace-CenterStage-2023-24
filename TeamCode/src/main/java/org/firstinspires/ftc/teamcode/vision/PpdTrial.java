package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class PpdTrial  {
    private PurplePixelDetection pipe;
    private OpenCvWebcam web;
    private Telemetry telemetry;

    public PpdTrial(LinearOpMode op)
    {
        web = OpenCvCameraFactory.getInstance().createWebcam(
                op.hardwareMap.get(WebcamName.class,"webcam")
        );
        telemetry = op.telemetry;
    }

    public void findStuff()
    {
        pipe = new PurplePixelDetection(telemetry);

        web.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                web.setPipeline(pipe);
                web.startStreaming(680, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error:", "Camera produced error code: " + errorCode);
                stop();
            }
        });
    }

    public void stop()
    {
        web.stopStreaming();
    }
}
