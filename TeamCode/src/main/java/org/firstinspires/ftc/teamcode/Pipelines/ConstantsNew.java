package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class ConstantsNew {
    public static final Rect LEFT_ROI_RED = new Rect(
            new Point(50, 140),
            new Point(200, 300)
    );
    public static final Rect CENTER_ROI_RED = new Rect(
            new Point(50, 120),
            new Point(170, 260)
    );
    public static final Rect RIGHT_ROI_RED = new Rect(
            new Point(340, 140),
            new Point(470, 290)
    );
    public static final Rect LEFT_ROI_BLUE = new Rect(
            new Point(80, 150),
            new Point(240, 290)
    );
    public static final Rect CENTER_ROI_BLUE = new Rect(
            new Point(430, 130),
            new Point(540, 250)
    );
    public static final Rect RIGHT_ROI_BLUE = new Rect(
            new Point(50, 140),
            new Point(200, 300)
    );
    public static final Scalar RED_LOW_HSV = new Scalar(0, 160,  120);
    public static final Scalar RED_HIGH_HSV = new Scalar(30, 255, 255);
    public static final Scalar RED_LOW_HSV1 = new Scalar(120, 45, 0);
    public static final Scalar RED_HIGH_HSV1 = new Scalar(180, 255, 255);
    public static final Scalar BLUE_LOW_HSV = new Scalar(105, 85, 40);
    public static final Scalar BLUE_HIGH_HSV = new Scalar(115, 225, 225);
}
