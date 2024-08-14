package org.firstinspires.ftc.teamcode.tfod;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class DummyPipeline extends OpenCvPipeline {
    public Mat currentFrame = null;
    @Override
    public Mat processFrame(Mat input) {
        currentFrame = input;
        return input;
    }
}
