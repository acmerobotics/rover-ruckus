package com.acmerobotics.roverruckus.opMode.test;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name="fftuning")
@Config
public class FFTuning extends LinearOpMode{

    public static double ACCELERATION = 15.0;
    public static double F = 12.759;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double DISTANCE = 48;
    private Robot robot;

    @Override
    public void runOpMode()  {
        robot = new Robot(this, hardwareMap);
        double f = robot.drive.getMotorPIDF().f;
        robot.addTelemetry("f", f);
        robot.addTelemetry("p", robot.drive.getMotorPIDF().p);
        robot.addTelemetry("i", robot.drive.getMotorPIDF().i);
        robot.addTelemetry("d", robot.drive.getMotorPIDF().d);
        robot.drive.setMotorPIDF(P, I, D, F);

        FileWriter writer = null;
        try {
            String path = Environment.getExternalStorageDirectory().getPath() + "/fftuner/";
            new File(path).mkdirs();
            Log.e("path", path);
            writer = new FileWriter(path + System.currentTimeMillis() + ".csv");
            writer.write("target, actual\n");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        waitForStart();

        long startTime = System.currentTimeMillis();
        while (!isStopRequested()) {
//            double v = (System.currentTimeMillis() - startTime) / (ACCELERATION * F);
//            robot.drive.setRealVelocity(new Pose2d(v, 0, 0));
//            robot.addTelemetry("target", v);
            robot.addTelemetry("actual", robot.drive.getVelocity());
//            try {
//                writer.write(v + ", " + robot.drive.getVelocity() + "\n");
//            } catch (Exception e) {
//                e.printStackTrace();
//                Log.e("ahhhh", "ahhhhhhhhh");
//                return;
//            }
            robot.drive.setMotorPIDF(P, I, D, F);
            robot.drive.setCurrentEstimatedPose(new Pose2d());
            robot.drive.followPath(new PathBuilder(new Pose2d()).lineTo(new Vector2d(DISTANCE, 0), new ConstantInterpolator(0)).build());
            while (!isStopRequested() && robot.drive.isFollowingPath()) {
                robot.update();
            }
            robot.drive.followPath(new PathBuilder(new Pose2d(DISTANCE, 0, 0)).lineTo(new Vector2d(0, 0), new ConstantInterpolator(0)).build());
            while (!isStopRequested() && robot.drive.isFollowingPath()) {
                robot.update();
            }
        }

        try {
            writer.flush();
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
