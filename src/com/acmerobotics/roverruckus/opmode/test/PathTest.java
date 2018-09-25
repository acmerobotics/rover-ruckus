package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="pathTest")
@Config
public class PathTest extends LinearOpMode{

    public static double maxV = 50;
    public static double maxOmega = 10;

    @Override
    public void runOpMode() {
        MecanumDrive drive =  new MecanumDrive(hardwareMap);

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();

        List<Path> paths = new ArrayList<>();

        paths.add(new PathBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(24, 0), new ConstantInterpolator(0))
                .build());

        for (Path path: paths) {
            drive.followPath(path);
            while (drive.isFollowingPath()) {
                if (isStopRequested()) {
                    drive.stop();
                    return;
                }
                drive.update();

            }
        }


    }
}
