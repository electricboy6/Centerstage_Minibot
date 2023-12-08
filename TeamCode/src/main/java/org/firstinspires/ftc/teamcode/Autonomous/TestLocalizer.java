package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Environment;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AprilTag.Localizer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.LineNumberReader;
import java.math.BigInteger;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

@Autonomous
public class TestLocalizer extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() throws InterruptedException {

        Localizer localizer = new Localizer();
        localizer.init(hardwareMap);
        addToLog("END");
        moveLog();
        addToLog("START");

        waitForStart();
        while(opModeIsActive()) {
            addToLog(formatPose2d(localizer.getPosition()));
            sleep(50);
        }
    }
    private String formatPose2d(@NonNull Pose2d input) {
        return (input.getX() + "," + input.getY() + "," + Math.toDegrees(input.getHeading()));
    }
    private void moveLog() {
        File logFile = new File(Environment.getExternalStorageDirectory(), "robotPositions/robotPosition.txt");
        FileReader reader = null;
        int lineCount = 0;
        try {
            reader = new FileReader(logFile);
            LineNumberReader numberReader = new LineNumberReader(reader);
            numberReader.skip(Long.MAX_VALUE);
            lineCount = numberReader.getLineNumber();
            numberReader.close();
        } catch(IOException e) {
            System.err.println("Unable to read log file!");
        }

        char[] fileDataTemp = new char[lineCount];
        try {
            reader.read(fileDataTemp);
        } catch (IOException e) {
            System.err.println("Unable to read log file!");
        }
        byte[] fileData = String.valueOf(fileDataTemp).getBytes();
        byte[] hash = null;
        try {
            hash = MessageDigest.getInstance("MD5").digest(fileData);
        } catch (NoSuchAlgorithmException ignored) {}

        String fileHash = new BigInteger(1, hash).toString(16);

        File newFile = new File(Environment.getExternalStorageDirectory(), "robotPositions/robotPosition-" + fileHash + ".txt");
        if (logFile.exists()) {
            if (!logFile.renameTo(newFile)) {
                System.err.println("Unable to rename old log!");
            }
        }
    }
    private void addToLog(String data) {
        File logFile = new File(Environment.getExternalStorageDirectory(), "robotPositions/robotPosition.txt");
        if(!logFile.exists()) {
            try {
                logFile.createNewFile();
            } catch (IOException e) {
                System.err.println("Unable to create log!");
            }
        }
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(logFile, true));
            writer.append(data);
            writer.newLine();
            writer.close();
        } catch (IOException e) {
            System.err.println("Unable to write to log!");
        }
    }
}