package com.acmerobotics.roverruckus.opMode.test;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileWriter;

public class TrackerTuning extends LinearOpMode {

    @Override
    public void runOpMode () {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            double theta = Math.PI / 2;
            Vector2d start = robot.drive.getTrackingPositions();

            robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getCurrentEstimatedPose()).);
        }

    }
}
