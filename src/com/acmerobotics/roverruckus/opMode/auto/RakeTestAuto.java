package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="rakeTestAuto")
@Config
public class RakeTestAuto extends AutoOpMode{

    public static double rakeX = 20;
    public static double rakeY = 0;

    @Override
    protected void run() {
        robot.drive.setCurrentEstimatedPose(new Pose2d());
        setRakePosition(new Vector2d(rakeX, rakeY));
        robot.waitForAllSubsystems();
        robot.pause(5000);
    }
}
