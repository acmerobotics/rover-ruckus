package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="colorTest")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robto = new Robot(this, hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
