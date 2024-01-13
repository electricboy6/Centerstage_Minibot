package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.PipelineNewCamera;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.Pipelines.VisionProcessorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RunVisionProcessorCam1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VisionProcessorPipeline detector = new VisionProcessorPipeline(telemetry, StartPosition.RED_STAGE);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(detector)
                .enableLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        visionPortal.setProcessorEnabled(detector, true);

        waitForStart();
        visionPortal.setProcessorEnabled(detector, false);
        visionPortal.stopLiveView();
        visionPortal.stopStreaming();
        visionPortal.close();
    }
}
