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

        robot.intake.extendToDeploy();

        robot.waitForAllSubsystems();

        robot.pause(500);

        robot.intake.deployMarker();

        robot.intake.retractRake();

        robot.waitForAllSubsystems();

    }
}
