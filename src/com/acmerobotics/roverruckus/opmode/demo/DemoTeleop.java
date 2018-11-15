package com.acmerobotics.roverruckus.opmode.demo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="demo", name="demo")
public class DemoTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        telemetry.addLine("Press Play to Start >");

        waitForStart();

        telemetry.clear();

        while (!isStopRequested()) {
            robot.drive.setVelocity(new Pose2d(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
                    ));
        }
    }

}
