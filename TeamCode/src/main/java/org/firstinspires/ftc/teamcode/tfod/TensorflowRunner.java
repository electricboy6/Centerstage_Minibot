package org.firstinspires.ftc.teamcode.tfod;

import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "tfod runner")
public class TensorflowRunner extends LinearOpMode {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        DummyPipeline frameCapture = initTFOD();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                processTFOD(frameCapture);
                telemetry.update();

                sleep(20);
            }
        }
        visionPortal.close();

    }
    private DummyPipeline initTFOD() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        DummyPipeline detector = new DummyPipeline();
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        // this camera supports 1280x800, 1280x720, 800x600, 640x480, and 320x240
                        webcam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE, OpenCvWebcam.StreamFormat.MJPEG);
                    }

                    @Override
                    public void onError(int errorCode) {
                        throw new RuntimeException("Unable to open camera!");
                    }
        });

        tfod = new TfodProcessor.Builder()
                .setModelFileName("/sdcard/model.tflite")
                .setModelLabels(new String[]{"pixel"})
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setUseObjectTracker(true)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(true);

        builder.addProcessor(tfod);

        visionPortal = builder.build();
        visionPortal.saveNextFrameRaw("frame");
        System.out.println("Saved frame");

        tfod.setMinResultConfidence(0.7f); // 70%

        // visionPortal.setProcessorEnabled(tfod, false);

        return detector;
    }

    private void processTFOD(DummyPipeline frameCapture) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.addData("FPS", visionPortal.getFps());

        assert frameCapture.currentFrame != null;

        System.out.println(Arrays.toString(PixelPipeline.getPixels(frameCapture.currentFrame, currentRecognitions)));

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}