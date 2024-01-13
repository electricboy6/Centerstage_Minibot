package org.firstinspires.ftc.teamcode.Pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.BLUE_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.BLUE_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.CENTER_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.CENTER_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.LEFT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.LEFT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.RED_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.RED_HIGH_HSV1;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.RED_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.RED_LOW_HSV1;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsNew.RIGHT_ROI_RED;

public class VisionProcessorPipeline implements VisionProcessor {
    Mat mat;
    Mat filteredFrame;
    Rect NONCENTER_ROI;
    Rect CENTER_ROI;
    Prop undetectableLocation;
    Prop detectableNoncenter;
    Scalar lowHSV;
    Scalar highHSV;
    Scalar lowHSV1;
    Scalar highHSV1;
    static final double PERCENT_COLOR_THRESHOLD = 15;
    Prop location;
    Telemetry telemetry;
    boolean telemetryEnabled = true;
    static Paint noncenterPaint = new Paint();
    static Paint centerPaint = new Paint();
    static {
        noncenterPaint.setStyle(Paint.Style.STROKE);
        centerPaint.setStyle(Paint.Style.STROKE);
    }
    public VisionProcessorPipeline(Telemetry t, StartPosition start) {
        if (start == StartPosition.RED_AUD) {
            NONCENTER_ROI = LEFT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            undetectableLocation = Prop.RIGHT;
            detectableNoncenter = Prop.LEFT;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
            lowHSV1 = RED_LOW_HSV1;
            highHSV1 = RED_HIGH_HSV1;
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
    public Object processFrame(@NonNull Mat frame, long captureTimeNanos) {
        mat = frame;
        filteredFrame = mat;
        if(frame.empty()) throw new CvException("Input mat was empty!");

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_BGR2HSV);
        Core.inRange(mat, lowHSV1, highHSV1, filteredFrame);
        Core.inRange(mat, lowHSV, highHSV, mat);
        //Core.add(mat, filteredFrame, mat);
        Core.bitwise_and(mat, filteredFrame, mat);

        Mat noncenter = mat.submat(NONCENTER_ROI); //sub matrices of mat
        Mat center = mat.submat(CENTER_ROI);

        // if a pixel is between the low and high HSV range OpenCV gives it a value of 255, or white.
        // This way the new image is just grayscale where 0 is black and 255 is white.
        // Below all elements of the ROI are added up and divided by the area and then by 2.55.
        // This essentially calculates the percentage of identified pixels.

        double noncenterValue = Core.sumElems(noncenter).val[0] / NONCENTER_ROI.area() / 2.55;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 2.55;

        boolean inNoncenterPosition = noncenterValue > PERCENT_COLOR_THRESHOLD; // sets a limit to compare to so small objects don't accidentally trigger
        boolean inCenterPosition = centerValue > PERCENT_COLOR_THRESHOLD;

        noncenter.release();
        center.release();

        if(inNoncenterPosition) {
            location = detectableNoncenter;
            noncenterPaint.setColor(Color.GREEN);
            centerPaint.setColor(Color.RED);
        } else if(inCenterPosition) {
            location = Prop.CENTER;
            centerPaint.setColor(Color.GREEN);
            noncenterPaint.setColor(Color.RED);
        } else {
            location = undetectableLocation;
            centerPaint.setColor(Color.RED);
            noncenterPaint.setColor(Color.RED);
        }

        if (telemetryEnabled) {
            telemetry.addData("Detected position: ", String.valueOf(location));
            telemetry.addData(detectableNoncenter + " percentage", Math.round(noncenterValue) + "%");
            telemetry.addData("CENTER percentage", Math.round(centerValue) + "%");
            telemetry.update();
        }

        mat = frame;
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_BGR2RGB);

        return mat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        noncenterPaint.setStrokeWidth(scaleCanvasDensity * 4);
        centerPaint.setStrokeWidth(scaleCanvasDensity * 4);

        android.graphics.Rect centerROI = makeGraphicsRect(CENTER_ROI, scaleBmpPxToCanvasPx);
        android.graphics.Rect noncenterROI = makeGraphicsRect(NONCENTER_ROI, scaleBmpPxToCanvasPx);

        Mat output = (Mat) userContext;
        Bitmap mat = Bitmap.createBitmap(output.cols(), output.rows(), Bitmap.Config.RGB_565);
        canvas.setBitmap(mat);
        output.release();

        canvas.drawRect(centerROI, centerPaint);
        canvas.drawRect(noncenterROI, noncenterPaint);
    }
    public Prop getPropLocation() {
        return location;
    }
    public void toggleTelemetry() {
        telemetryEnabled = !telemetryEnabled;
    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }
}
