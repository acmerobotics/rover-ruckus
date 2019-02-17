package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.JoystickTransform;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="drive")
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode () {
        Robot robot = new Robot(this, hardwareMap);
        JoystickTransform transform = new JoystickTransform();

        waitForStart();

        while (!isStopRequested()) {
            Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
            robot.drive.setVelocity(v);
            robot.update();
        }
    }
}
