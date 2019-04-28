package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "rakeTest")
public class RakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            robot.intake.setArmPower(gamepad1.left_trigger - gamepad1.right_trigger);
            telemetry.addData("rakePower", gamepad1.left_trigger - gamepad1.right_trigger);
            telemetry.update();
            robot.update();
        }

    }
}
