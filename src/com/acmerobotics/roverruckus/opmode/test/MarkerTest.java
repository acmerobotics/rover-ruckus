package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="markerTest")
public class MarkerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.a) robot.lift.markerDown();
            else robot.lift.markerUp();
            robot.update();
        }
    }
}
