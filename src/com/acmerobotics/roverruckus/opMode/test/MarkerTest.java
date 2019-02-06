package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "markerTest")
public class MarkerTest extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) robot.lift.markerDown();
            else if (gamepad1.b) robot.lift.markerUp();
            else if (gamepad1.x) {
                robot.lift.releaseMarker();
            }
        }
    }
}
