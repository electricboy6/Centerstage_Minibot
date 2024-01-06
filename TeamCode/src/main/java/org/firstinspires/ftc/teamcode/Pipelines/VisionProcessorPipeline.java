package org.firstinspires.ftc.teamcode.Pipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.Pipelines.Constants.BLUE_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.BLUE_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.CENTER_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.CENTER_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.LEFT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.LEFT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RED_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RED_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RIGHT_ROI_RED;

public class VisionProcessorPipeline implements VisionProcessor {
    Mat mat;
    Rect NONCENTER_ROI;
    Rect CENTER_ROI;
    Prop undetectableLocation;
    Prop detectableNoncenter;
    Scalar lowHSV;
    Scalar highHSV;
    double PERCENT_COLOR_THRESHOLD = 0.15;
    Prop location;
    Telemetry telemetry;
    boolean telemetryEnabled = true;
    public VisionProcessorPipeline(Telemetry t, StartPosition start) {
        if (start == StartPosition.RED_AUD) {
            NONCENTER_ROI = LEFT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            undetectableLocation = Prop.RIGHT;
            detectableNoncenter = Prop.LEFT;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
        }
        else if (start == StartPosition.BLUE_AUD) {
            NONCENTER_ROI = LEFT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
            undetectableLocation = Prop.LEFT;
            detectableNoncenter = Prop.RIGHT;
            lowHSV = BLUE_LOW_HSV;
            highHSV = BLUE_HIGH_HSV;
        }
        else if (start == StartPosition.RED_STAGE) {
            NONCENTER_ROI = RIGHT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            undetectableLocation = Prop.LEFT;
            detectableNoncenter = Prop.RIGHT;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
        }
        else if (start == StartPosition.BLUE_STAGE) {
            NONCENTER_ROI = LEFT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
            undetectableLocation = Prop.RIGHT;
            detectableNoncenter = Prop.LEFT;
            lowHSV = BLUE_LOW_HSV;
            highHSV = BLUE_HIGH_HSV;
        }
        else {
            throw new IllegalArgumentException("Invalid start position passed to pipeline!");
        }
        telemetry = t;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //Imgproc.resize(frame, mat, new Size(384, 240));
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat noncenter = mat.submat(NONCENTER_ROI); //sub matrices of mat
        Mat right = mat.submat(CENTER_ROI);

        // if a pixel is deemed to be between the low and high HSV range OpenCV makes it white
        // white is given a value of 255. This way the new image is just grayscale where 0 is black
        // and 255 is white. Below all elements of the sub matrix are added up and divided by the area and then by 255.
        // This essentially calculates the ratio of identified pixels to those not identified. The
        //higher the value the more detection.

        double noncenterValue = Core.sumElems(noncenter).val[0] / NONCENTER_ROI.area() / 255;
        double centerValue = Core.sumElems(right).val[0] / CENTER_ROI.area() / 255;

        noncenter.release(); // frees up memory
        right.release();

        boolean inNoncenterPosition = noncenterValue > PERCENT_COLOR_THRESHOLD; // sets a limit to compare to so small objects don't accidentally trigger
        boolean inCenterPosition = centerValue > PERCENT_COLOR_THRESHOLD;
        location = undetectableLocation;
        if(inNoncenterPosition) location = detectableNoncenter;
        else if(inCenterPosition) location = Prop.CENTER;

        if (telemetryEnabled) {
            telemetry.addData("Detected position: ", String.valueOf(getPropLocation()));
            telemetry.addData(detectableNoncenter + " percentage", Math.round(noncenterValue * 100) + "%");
            telemetry.addData("CENTER percentage", Math.round(centerValue * 100) + "%");
            telemetry.update();
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, NONCENTER_ROI, location == detectableNoncenter? colorTSE:colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, CENTER_ROI, location == Prop.CENTER? colorTSE:colorNone);

        return mat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public Prop getPropLocation() {
        return location;
    }
    public void toggleTelemetry() {
        telemetryEnabled = !telemetryEnabled;
    }
}
