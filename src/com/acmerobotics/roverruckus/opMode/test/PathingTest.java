package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.SplineTrajectory;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Config
@Autonomous(name = "pathTest")
public class PathingTest extends LinearOpMode {

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        robot.drive.setCurrentEstimatedPose(new Pose2d());
        SplineTrajectory.AXIAL_P = 0;
        SplineTrajectory.HEADING_P = 0;
        SplineTrajectory.LATERAL_P = 0;

        waitForStart();


        while (opModeIsActive()) {
            robot.drive.setMotorPIDF(P, I, D, MecanumDrive.F);
            ArrayList<Trajectory> trajectories = new TrajectoryBuilder(new Waypoint(robot.drive.getCurrentEstimatedPose(), 0))
                .to(new Waypoint(new Pose2d(24, 0, Math.PI), 0, Math.PI / 2))
                .to(new Waypoint(new Pose2d(24, 24, 0), Math.PI / 2, Math.PI))
                .to(new Waypoint(new Pose2d(), -Math.PI / 2))
                .build();

            for (Trajectory trajectory: trajectories) {
                 robot.drive.followTrajectory(trajectory);
                 robot.waitForAllSubsystems();
            }

        }

    }

}
