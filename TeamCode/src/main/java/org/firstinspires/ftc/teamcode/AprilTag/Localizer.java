package org.firstinspires.ftc.teamcode.AprilTag;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.List;

public class Localizer {
    private AprilTagProcessor processor = new AprilTagProcessor.Builder()
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            .build();
    private VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
    private VisionPortal visionPortal;
    private IMU imu;
    private static final HashMap<Integer, Pose2d> aprilTagPositions = new HashMap<>();
    static {
        aprilTagPositions.put(1, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(2, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(3, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(4, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(5, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(6, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(7, new Pose2d(
                0, 0
        ));
        aprilTagPositions.put(8, new Pose2d(
                0, 0
        ));
    }

    public void init(HardwareMap hwMap) {
        visionPortalBuilder.addProcessor(processor);
        visionPortal = visionPortalBuilder.build();
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(imuParameters);
    }
    public Pose2d getPosition() {
        JSONObject main = new JSONObject();
        List<AprilTagDetection> detections = processor.getDetections();
        for(AprilTagDetection detection : detections) {
            try {
                main.put(String.valueOf(detection.id), (detection.ftcPose.yaw + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            }
            catch (JSONException ignored) {}
        }
        return new Pose2d();
    }
    private Pose2d runTriangulation(Pose2d pos1, Pose2d pos2) {
        Pose2d output = new Pose2d(
                ((pos1.getY() - pos2.getY()) + (pos2.getX() * Math.tan(pos2.getHeading())
                        - (pos1.getX() * Math.tan(pos1.getHeading())))
                        / (Math.tan(pos2.getHeading()) - Math.tan(pos1.getHeading()))),
                ((pos1.getY() * Math.tan(pos2.getHeading())) - (pos2.getY() * Math.tan(pos1.getHeading()))
                        - ((pos1.getX() - pos2.getX()) * Math.tan(pos2.getHeading()))
                        / (Math.tan(pos2.getHeading()) - Math.tan(pos1.getHeading())))
        );
        return output;
    }
}