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

    public static double P = MecanumDrive.P;
    public static double I = MecanumDrive.I;
    public static double D = MecanumDrive.D;
    public static double F = MecanumDrive.F;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        robot.drive.setCurrentEstimatedPose(new Pose2d());

        waitForStart();


        while (opModeIsActive()) {
            robot.drive.setMotorPIDF(P, I, D, F);
            ArrayList<Trajectory> trajectories = new TrajectoryBuilder(new Waypoint(robot.drive.getCurrentEstimatedPose(), 0))
                .to(new Waypoint(new Pose2d(24, 0, Math.PI), 0, Math.PI / 2))
                .to(new Waypoint(new Pose2d(24, 24, 0), Math.PI / 2, Math.PI))
                .to(new Waypoint(new Pose2d(), -Math.PI / 2))
//                    .to(new Waypoint(new Pose2d(48, 0, 0), 0, Math.PI))
//                    .to(new Waypoint(new Pose2d(0, 0, 0), Math.PI))
                .build();

            for (Trajectory trajectory: trajectories) {
                 robot.drive.followTrajectory(trajectory);
                 robot.waitForAllSubsystems();
            }

        }

    }

}
