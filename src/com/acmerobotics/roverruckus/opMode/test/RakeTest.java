package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "rakeTest")
@Config
public class RakeTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        robot.intake.retractRake();
        robot.waitForAllSubsystems();

        while (opModeIsActive()) robot.update();
    }
}
