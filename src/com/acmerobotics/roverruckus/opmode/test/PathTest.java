package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="pathTest")
@Config
public class PathTest extends LinearOpMode{


    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();

        List<Path> paths = new ArrayList<>();

//        paths.add(new PathBuilder(new Pose2d(0,0,0))
//                .lineTo(new Vector2d(24, 0), new ConstantInterpolator(0))
//                .build());
//
//        paths.add(new PathBuilder(paths.get(0).end())
//                .lineTo(new Vector2d(-24, 0), new LinearInterpolator(0, Math.PI/2))
//                .build());

        paths.add(new PathBuilder(new Pose2d(0,0,0))
                .splineTo(new Pose2d(50, 24, 0), new ConstantInterpolator(0))
                .splineTo(new Pose2d(90, 36, Math.PI/2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(50, 72, Math.PI), new ConstantInterpolator(0))
                .splineTo(new Pose2d(36, 36, -Math.PI/2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(12, 36, Math.PI/2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(12, 90, Math.PI/2), new ConstantInterpolator(0))
                .build());

        for (Path path: paths) {
            robot.drive.followPath(path);
            while (robot.drive.isFollowingPath()) {
                if (isStopRequested()) {
                    robot.drive.stop();
                    return;
                }
            }
        }


    }
}
