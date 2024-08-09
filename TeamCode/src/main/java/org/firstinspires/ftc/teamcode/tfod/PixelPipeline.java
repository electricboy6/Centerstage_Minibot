package org.firstinspires.ftc.teamcode.tfod;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class PixelPipeline {
    private static Mat mat = new Mat();
    private static Mat white = new Mat();
    private static Mat yellow = new Mat();
    private static Mat green = new Mat();
    private static Mat purple = new Mat();
    private static double whitePercent;
    private static double yellowPercent;
    private static double greenPercent;
    private static double purplePercent;
    private static final int MIN_COLOR_PERCENT = 50;
    public static Pixel[] getPixels(Mat input, List<Recognition> pixels) {
        Pixel[] output = new Pixel[pixels.size()];
        Pixel.PIXEL_COLORS color = Pixel.PIXEL_COLORS.NONE;
        double area = 0;
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        int i = 0;
        for(Recognition pixel_rec : pixels) {
            double x = (pixel_rec.getLeft() + pixel_rec.getRight()) / 2 ;
            double y = (pixel_rec.getTop()  + pixel_rec.getBottom()) / 2 ;
            Rect pixel = new Rect(
                    new Point(pixel_rec.getLeft(), pixel_rec.getTop()),
                    new Point(pixel_rec.getRight(), pixel_rec.getBottom())
            );
            Core.inRange(mat, PixelColors.WHITE_LOW_HSV, PixelColors.WHITE_HIGH_HSV, white);
            Core.inRange(mat, PixelColors.YELLOW_LOW_HSV, PixelColors.YELLOW_HIGH_HSV, yellow);
            Core.inRange(mat, PixelColors.GREEN_LOW_HSV, PixelColors.GREEN_HIGH_HSV, green);
            Core.inRange(mat, PixelColors.PURPLE_LOW_HSV, PixelColors.PURPLE_HIGH_HSV, purple);
            whitePercent = Core.sumElems(white).val[0] / pixel.area() / 2.55;
            yellowPercent = Core.sumElems(yellow).val[0] / pixel.area() / 2.55;
            greenPercent = Core.sumElems(green).val[0] / pixel.area() / 2.55;
            purplePercent = Core.sumElems(purple).val[0] / pixel.area() / 2.55;
            if(whitePercent > Math.max(Math.max(yellowPercent, greenPercent), purplePercent)) {
                color = Pixel.PIXEL_COLORS.WHITE;
                area = whitePercent;
            } else if(yellowPercent > Math.max(Math.max(whitePercent, greenPercent), purplePercent)) {
                color = Pixel.PIXEL_COLORS.WHITE;
                area = yellowPercent;
            } else if(greenPercent > Math.max(Math.max(yellowPercent, whitePercent), purplePercent)) {
                color = Pixel.PIXEL_COLORS.WHITE;
                area = greenPercent;
            } else if(purplePercent > Math.max(Math.max(yellowPercent, greenPercent), whitePercent)) {
                color = Pixel.PIXEL_COLORS.WHITE;
                area = purplePercent;
            } // todo: fix undesired behavior when two+ percentages are identical
            if(area < MIN_COLOR_PERCENT) {
                color = Pixel.PIXEL_COLORS.NONE;
            }
            output[i] = new Pixel(color, pixel);
            i++;
        }
        return output;
    }
}
