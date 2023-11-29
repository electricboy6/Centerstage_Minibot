package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class SquareToTag {
    private AprilTagProcessor processor = new AprilTagProcessor.Builder()
            // these values are for calibration
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            .build();
    private VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
    private VisionPortal visionPortal;

    public void init(@NonNull HardwareMap hwMap) {
        visionPortalBuilder.addProcessor(processor);
        visionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        visionPortal = visionPortalBuilder.build();
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
    }

    private HashMap<Integer, Double> main = new HashMap<>();

    public double getPosition(int tagID) {
        List<AprilTagDetection> detections = processor.getDetections();
        double angle;
        for (AprilTagDetection detection : detections) {
            if (detection != null && detection.id == tagID) {
                angle = Math.toRadians(detection.ftcPose.bearing);
                main.put(detection.id, angle);
            }
        }

        return 0;
    }
    private void safeWait(long ms) {
        try {
            wait(ms);
        } catch(InterruptedException | IllegalMonitorStateException ignored) {}
    }
}