package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.PipelineNewCamera;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous
public class RunPipelineCam1 extends LinearOpMode {
    OpenCvWebcam webcam; // If it is an OpenCvCamera, you can't set the stream format.
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        PipelineNewCamera detector = new PipelineNewCamera(telemetry, StartPosition.RED_STAGE);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // this camera supports 1280x800, 1280x720, 800x600, 640x480, and 320x240
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE, OpenCvWebcam.StreamFormat.MJPEG);
            }
            @Override
            public void onError(int errorCode) {}

        });
        waitForStart();
    }
}
