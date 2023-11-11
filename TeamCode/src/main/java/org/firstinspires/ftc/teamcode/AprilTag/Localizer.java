package org.firstinspires.ftc.teamcode.AprilTag;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
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
                -42, 62.5
        ));
        aprilTagPositions.put(2, new Pose2d(
                -36, 62.5
        ));
        aprilTagPositions.put(3, new Pose2d(
                -30, 62.5
        ));
        aprilTagPositions.put(4, new Pose2d(
                30, 62.5
        ));
        aprilTagPositions.put(5, new Pose2d(
                36, 62.5
        ));
        aprilTagPositions.put(6, new Pose2d(
                42, 62.5
        ));
        /* for bigger tag
        aprilTagPositions.put(7, new Pose2d(
                0, -72
        ));
         */
        aprilTagPositions.put(8, new Pose2d(
                36, -72
        ));
        aprilTagPositions.put(9, new Pose2d(
                -36, -72
        ));
        /* for bigger tag
        aprilTagPositions.put(10, new Pose2d(
            0, -72
        ));
         */
    }
    Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetryInput) {
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
        telemetry = telemetryInput;
    }
    private HashMap<Integer, Double> main;
    public Pose2d getPosition() {
        main = new HashMap<Integer, Double>();
        List<AprilTagDetection> detections = processor.getDetections();
        for(AprilTagDetection detection : detections) {
            System.out.println("id " + detection.id + ", angle " + (detection.ftcPose.yaw + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            main.put(detection.id, (detection.ftcPose.yaw + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        }
        ArrayList<Pose2d> triangulatedPositions = new ArrayList<>();
        Object[] keys = main.keySet().toArray();
        for(int i = 0; i < main.size(); i++) {
            for(int j = 0; j < main.size(); j++) {
                System.out.println(keys[i]);
                triangulatedPositions.add(runTriangulation(addHeadingToPose2d(aprilTagPositions.getOrDefault((int) keys[i], new Pose2d(0, 0, 0)), (int) keys[i]),
                        addHeadingToPose2d(aprilTagPositions.getOrDefault((int) keys[j], new Pose2d(0, 0, 0)), main.get((int) keys[j]))));
            }
        }
        double averageX = 0;
        double averageY = 0;
        for(Pose2d position : triangulatedPositions) {
            averageX += position.getX();
            averageY += position.getY();
        }
        averageX = averageX / triangulatedPositions.size();
        averageY = averageY / triangulatedPositions.size();
        return new Pose2d(
                averageX, averageY, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }
    private Pose2d addHeadingToPose2d(Pose2d input, double heading) {
        return new Pose2d(
                input.getX(),
                input.getY(),
                heading
        );
    }
    private Pose2d runTriangulation(Pose2d pos1, Pose2d pos2) {
        double tanPos1 = Math.tan(pos1.getHeading()); // for optimization
        double tanPos2 = Math.tan(pos2.getHeading());
        // start of triangulation
        return new Pose2d(
                // triangulate x coordinate
                ((pos1.getY() - pos2.getY()) + (pos2.getX() * tanPos2
                        - (pos1.getX() * tanPos1))
                        / (tanPos2 - tanPos1)),
                // triangulate y coordinate
                ((pos1.getY() * tanPos2) - (pos2.getY() * tanPos1)
                        - ((pos1.getX() - pos2.getX()) * tanPos2)
                        / (tanPos2 - tanPos1))
        );
    }
}