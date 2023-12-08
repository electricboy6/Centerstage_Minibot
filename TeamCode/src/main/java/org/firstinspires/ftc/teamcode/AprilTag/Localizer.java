package org.firstinspires.ftc.teamcode.AprilTag;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class Localizer {
    private AprilTagProcessor processor = new AprilTagProcessor.Builder()
            // these values are for calibration
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            .build();
    private VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
    private VisionPortal visionPortal;
    private IMU imu;
    private static final int xOffset = 0;
    private static final int yOffset = 0;
    private static final int headingOffset = 0;
    private static final float xMult = 1f;
    private static final float yMult = 1f;
    private static final float headingMult = 1f;
    private static final HashMap<Integer, Pose2d> aprilTagPositions = new HashMap<>();
    static {
        aprilTagPositions.put(1, new Pose2d(
                -42,62.5, Math.toRadians(90)
        ));
        aprilTagPositions.put(2, new Pose2d(
                -36,62.5, Math.toRadians(90)
        ));
        aprilTagPositions.put(3, new Pose2d(
                -30,62.5, Math.toRadians(90)
        ));
        aprilTagPositions.put(4, new Pose2d(
                30,62.5, Math.toRadians(90)
        ));
        aprilTagPositions.put(5, new Pose2d(
                36,62.5, Math.toRadians(90)
        ));
        aprilTagPositions.put(6, new Pose2d(
                42,62.5, Math.toRadians(90)
        ));
        aprilTagPositions.put(7, new Pose2d(
                42,-72, Math.toRadians(270)
        ));
        aprilTagPositions.put(8, new Pose2d(
                36,-72, Math.toRadians(270)
        ));
        aprilTagPositions.put(9, new Pose2d(
                -36,-72, Math.toRadians(270)
        ));
        aprilTagPositions.put(10, new Pose2d(
                -42,-72, Math.toRadians(270)
        ));
    }
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
    private HashMap<Integer, Double> main = new HashMap<>();
    public Pose2d getPosition() {
        List<AprilTagDetection> detections = processor.getDetections();
        if(detections.size() < 2) return null;
        double yaw = 0;
        double angle;
        for(AprilTagDetection detection : detections) {
            if(detection != null) { // HUGE paranoia
                yaw = aprilTagPositions.get(detection.id).getHeading() - Math.toRadians(detection.ftcPose.yaw);
                angle = aprilTagPositions.get(detection.id).getHeading() - Math.toRadians(detection.ftcPose.bearing);

                main.put(detection.id, (angle + aprilTagPositions.get(detection.id).getHeading()));
            }
        }
        ArrayList<Pose2d> triangulatedPositions = new ArrayList<>();
        ArrayList<int[]> testedPositions = new ArrayList<>();
        Object[] keys = main.keySet().toArray();
        for(int i = 0; i < main.size(); i++) {
            for(int j = 0; j < main.size(); j++) {
                if (
                        !((int) keys[i] == (int) keys[j])
                        && !(
                                testedPositions.contains(new int[]{(int) keys[i], (int) keys[j]}) ||
                                testedPositions.contains(new int[]{(int) keys[j], (int) keys[i]})
                )) {
                    testedPositions.add(new int[]{(int) keys[i], (int) keys[j]});
                    triangulatedPositions.add(
                            runTriangulation(addHeadingToPose2d(Objects.requireNonNull(aprilTagPositions.get((int) keys[i])), main.get((int) keys[i])),
                            addHeadingToPose2d(Objects.requireNonNull(aprilTagPositions.get((int) keys[j])), main.get((int) keys[j])))
                    );
                }
            }
        }

        return adjustPose2dForWebcamPosition(fixReturnedPosition(addHeadingToPose2d(averagePose2dList(triangulatedPositions), yaw)));
    }
    private Pose2d addHeadingToPose2d(@NonNull Pose2d input, double heading) {
        return new Pose2d(
                input.getX(),
                input.getY(),
                heading
        );
    }
    private Pose2d adjustPose2dForWebcamPosition(@NonNull Pose2d input) {
        return new Pose2d(input.getX() - xOffset, input.getY() - yOffset, input.getHeading() - headingOffset);
    }
    private Pose2d fixReturnedPosition(@NonNull Pose2d input) {
        //return new Pose2d(input.getX() * xMult, input.getY() * yMult, input.getHeading() * headingMult);
        return new Pose2d(axSquaredPlusBxPlusC(input.getX(), 0.04948462, 0.35136818, -37.875619), input.getY(), input.getHeading());
    }
    private double axSquaredPlusBxPlusC(double value, double a, double b, double c) {
        return (a * Math.pow(value, 2)) + (b * value) + c;
    }
    private Pose2d averagePose2dList(@NonNull ArrayList<Pose2d> input) {
        double averageX = 0;
        double averageY = 0;
        double averageHeading = 0;
        for(Pose2d currentPose2d : input) {
            averageX += (currentPose2d.getX());
            averageY += (currentPose2d.getY());
            averageHeading += (currentPose2d.getHeading());
        }
        averageX /= input.size();
        averageY /= input.size();
        averageHeading /= input.size();

        // averageX = input.size() / averageX;
        // averageY = input.size() / averageY;
        // averageHeading = input.size() / averageHeading;

        return new Pose2d(averageX, averageY, averageHeading);
    }
    private Pose2d runTriangulation(@NonNull Pose2d pos1, @NonNull Pose2d pos2) {
        double tanPos1 = Math.tan(pos1.getHeading());
        double tanPos2 = Math.tan(pos2.getHeading());
        double pos1X = pos1.getX();
        double pos1Y = pos1.getY();
        double pos2X = pos2.getX();
        double pos2Y = pos2.getY();
        // start of triangulation
        return new Pose2d(
                // triangulate x coordinate
                (((pos1Y - pos2Y) + (pos2X * tanPos2 - (pos1X * tanPos1)))
                        / (tanPos2 - tanPos1)),
                // triangulate y coordinate
                ((((pos1Y * tanPos2) - (pos2Y * tanPos1)) - ((pos1X - pos2X) * tanPos2))
                        / (tanPos2 - tanPos1))
        );
    }
}