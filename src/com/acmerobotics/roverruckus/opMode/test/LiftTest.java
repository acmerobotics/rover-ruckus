package com.acmerobotics.roverruckus.opMode.test;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.Differentiator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileWriter;

@Config
@TeleOp(name = "liftTest")
public class LiftTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        Differentiator diff = new Differentiator();

        waitForStart();

        while (opModeIsActive()) {
            double command = -gamepad1.left_stick_y;
            robot.lift.setPower(command);
            diff.update(robot.lift.getPosition());
            robot.addTelemetry("command", command);
            robot.addTelemetry("x", diff.getX());
            robot.addTelemetry("v", diff.getV());
            robot.addTelemetry("a", diff.getA());
            robot.addTelemetry("j", diff.getJ());
            robot.addTelemetry("frame", robot.lift.frameIsAtBottom());
            robot.addTelemetry("bottom", robot.lift.isAtBottom());
            robot.addTelemetry("latch", robot.lift.isAtLatch());
            robot.update();
        }

    }
}
