package org.firstinspires.ftc.teamcode.AprilTag;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
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
    private HashMap<Integer, Pose2d> main = new HashMap<>();
    public Pose2d getPosition() {
        List<AprilTagDetection> detections = processor.getDetections();
        if(detections.size() < 2) return null;
        //double yaw = 0;
        double angle;
        double[] yaws = new double[detections.size()];
        short i = 0;
        for(AprilTagDetection detection : detections) {
            if(detection != null) { // HUGE paranoia
                VectorF tagPos = detection.metadata.fieldPosition;
                yaws[i] = angles[detection.id] - Math.toRadians(detection.ftcPose.yaw);
                angle = angles[detection.id] - Math.toRadians(detection.ftcPose.bearing);

                main.put(detection.id, new Pose2d(tagPos.getData()[1], tagPos.getData()[0],angle + angles[detection.id]));
                i++;
            }
        }
        ArrayList<Pose2d> triangulatedPositions = new ArrayList<>();
        Object[] keys = main.keySet().toArray();
        for(i = 0; i < main.size(); i++) {
            for(short j = 0; j < main.size(); j++) {
                if (!((int) keys[i] == (int) keys[j])) {
                    triangulatedPositions.add(
                            addHeadingToPose2d(runTriangulation(main.get((int) keys[i]), main.get((int) keys[j])), yaws[i])
                    );
                }
            }
        }

        return adjustPose2dForWebcamPosition(fixReturnedPosition(averagePose2dList(triangulatedPositions)));
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
        return new Pose2d(input.getX() * xMult, input.getY() * yMult, input.getHeading() * headingMult);
        //return new Pose2d(axSquaredPlusBxPlusC(input.getX(), 0.04948462, 0.35136818, -37.875619), input.getY(), input.getHeading());
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
        double tanPos1Heading = Math.tan(pos1.getHeading());
        double tanPos2Heading = Math.tan(pos2.getHeading());
        double pos1X = pos1.getX();
        double pos1Y = pos1.getY();
        double pos2X = pos2.getX();
        double pos2Y = pos2.getY();
        // start of triangulation
        return new Pose2d(
                // triangulate x coordinate
                (((pos1Y - pos2Y) + (pos2X * tanPos2Heading - (pos1X * tanPos1Heading)))
                        / (tanPos2Heading - tanPos1Heading)),
                // triangulate y coordinate
                ((((pos1Y * tanPos2Heading) - (pos2Y * tanPos1Heading)) - ((pos1X - pos2X) * tanPos2Heading))
                        / (tanPos2Heading - tanPos1Heading))
        );
    }
}