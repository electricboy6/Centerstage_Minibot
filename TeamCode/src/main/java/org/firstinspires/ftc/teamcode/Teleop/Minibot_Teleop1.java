package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;


@Config
@TeleOp(group = "Teleop")

public class Minibot_Teleop1 extends LinearOpMode {
// test

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime turnerTimer = new ElapsedTime();

    //PixelDropper pixelDropper = new PixelDropper(this)

    private ElapsedTime teleopTimer = new ElapsedTime();
    private double TELEOP_TIME_OUT = 140; // WARNING: LOWER FOR OUTREACH


    FtcDashboard dashboard;


    //ENUMS

    @Override
    public void runOpMode() throws InterruptedException {


        // set up local variables
        double  speedFactor = 1.0;
        double expo =   3; // has to be 1 or 3

        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();

        while (!isStopRequested() && teleopTimer.time() < TELEOP_TIME_OUT) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            Math.pow(-gamepad1.left_stick_y, expo) * speedFactor,
                            Math.pow(-gamepad1.left_stick_x,expo) * speedFactor,
                            Math.pow(-gamepad1.right_stick_x,expo) * speedFactor
                    )

            );

        }
    }

}