package com.acmerobotics.roverruckus.opMode.test;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="trackerTuning")
public class TrackerTuning extends LinearOpMode {

    public static String TAG = "TrackerTuning";

    @Override
    public void runOpMode () {
        Robot robot = new Robot(this, hardwareMap);

        ArrayList<Double> x = new ArrayList<>();
        ArrayList<Double> y = new ArrayList<>();

        waitForStart();

        int ignore = 4;
        while (!isStopRequested()) {
            double theta = Math.PI / 2;
            robot.drive.setCurrentEstimatedPose(new Pose2d());
            Vector2d start = robot.drive.getTrackingPositions();

            robot.drive.followTrajectory(new TrajectoryBuilder(
                    new Waypoint(robot.drive.getCurrentEstimatedPose(), 0))
                    .turnTo(Angle.norm(robot.drive.getCurrentEstimatedPose().getHeading() + theta))
                    .build().get(0));

            robot.waitForAllSubsystems();

            if (ignore > 0) {
                ignore --;
                continue;
            }

            theta = Angle.norm(robot.drive.getCurrentEstimatedPose().getHeading());
            Log.i(TAG, "heading: " + theta);
            Vector2d end = robot.drive.getTrackingPositions();
            Vector2d delta = end.minus(start).div(theta);

            Log.i(TAG, delta.toString());
            x.add(delta.getX());
            y.add(delta.getY());

            Log.i(TAG, "avgX: " + average(x));
            Log.i(TAG, "avgY: " + average(y));

        }

    }

    public double average(List<Double> vals) {
        if (vals.size() < 1) return 0;
        double sum = 0;
        for (double val: vals) {
            sum += val;
        }
        return sum / vals.size();
    }
}
