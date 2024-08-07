package org.firstinspires.ftc.teamcode.tfod;

import org.opencv.core.Rect;

public class Pixel {
    public final PIXEL_COLORS color;
    public final Rect position;
    public enum PIXEL_COLORS {
        WHITE,
        YELLOW,
        GREEN,
        PURPLE,
        NONE
    }
    public Pixel(PIXEL_COLORS color, Rect position) {
        this.color = color;
        this.position = position;
    }
    public Pixel() {
        this.color = PIXEL_COLORS.NONE;
        this.position = null;
    }
}
