package com.acmerobotics.roverruckus.opMode.test;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.Differentiator;
import com.acmerobotics.roverruckus.util.LogWriter;
import com.acmerobotics.roverruckus.util.PoseUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileWriter;

@Autonomous(name = "fftuning")
@Config
public class FFTuning extends LinearOpMode {

    public static double ACCELERATION = .05;

    private Pose2d theOneWereMeasuring = new Pose2d(0, 1, 0);

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Differentiator diff = new Differentiator();
        diff.update(0);
        LogWriter writer = new LogWriter("driveLateral", "command", "velocity");

        waitForStart();

        double lastCommand = 0;
        while (!isStopRequested()) {
            double x = robot.drive.getCurrentEstimatedPose().getY();
            diff.update(x);
            robot.addTelemetry("ffTuningX", x);
            robot.addTelemetry("ffTuningV", diff.getV());
            robot.addTelemetry("ffTuningC", lastCommand);
            writer.writeLine(lastCommand, diff.getV());
            lastCommand += diff.getDt() * ACCELERATION;
            lastCommand = Range.clip(lastCommand, 0, 1);
            robot.drive.setPower(theOneWereMeasuring.times(lastCommand));
            robot.update();
        }

        writer.close();

    }
}
