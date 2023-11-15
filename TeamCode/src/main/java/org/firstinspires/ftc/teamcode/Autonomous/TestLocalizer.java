package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AprilTag.Localizer;

@Autonomous
public class TestLocalizer extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() throws InterruptedException {

        Localizer localizer = new Localizer();
        localizer.init(hardwareMap);

        waitForStart();

        System.out.println(localizer.getPosition());
    }
}