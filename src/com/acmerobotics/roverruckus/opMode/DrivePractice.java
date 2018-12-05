package com.acmerobotics.roverruckus.opMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.JoystickTransform;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DrivePractice")
public class DrivePractice extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        JoystickTransform transform = new JoystickTransform();

        waitForStart();

        while (opModeIsActive()) {
            robot.drive.setVelocity(new Pose2d(transform.transform(new Vector2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y), robot), -gamepad1.right_stick_x));

            if (gamepad1.left_bumper) transform.setRamping(true);
            else if (gamepad1.right_bumper) transform.setRamping(false);
            robot.addTelemetry("Ramping", transform.getRamping());

            if (gamepad1.a) transform.setMode(JoystickTransform.MODE.LINEAR);
            else if (gamepad1.b) transform.setMode(JoystickTransform.MODE.DUAL_ZONE);
            else if (gamepad1.x) transform.setMode(JoystickTransform.MODE.EXPONENTIAL);
            robot.addTelemetry("Mode", transform.getMode().toString());
            robot.update();
        }
    }

}
