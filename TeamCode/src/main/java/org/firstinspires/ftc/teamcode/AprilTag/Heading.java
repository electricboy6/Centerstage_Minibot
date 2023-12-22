package org.firstinspires.ftc.teamcode.AprilTag;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Heading {
    private static IMU imu;
    private static final double[] angles = new double[] {
            -0,
            Math.toRadians(90),
            Math.toRadians(90),
            Math.toRadians(90),
            Math.toRadians(90),
            Math.toRadians(90),
            Math.toRadians(90),
            Math.toRadians(270),
            Math.toRadians(270),
            Math.toRadians(270),
            Math.toRadians(270),
    };
    private static AprilTagProcessor processor = new AprilTagProcessor.Builder()
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
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(imuParameters);
        imu.resetYaw();
    }
    public static double getHeading() {
        List<AprilTagDetection> detections = processor.getDetections();
        if(detections.size() == 0) return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double yaw = 0;
        short i = 0;
        for(AprilTagDetection detection : detections) {
            if(detection != null) { // HUGE paranoia
                yaw += angles[detection.id] - Math.toRadians(detection.ftcPose.yaw);
                i++;
            }
        }
        yaw /= i;
        return yaw;
    }
    public static double getXangularVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate;
    }
}
