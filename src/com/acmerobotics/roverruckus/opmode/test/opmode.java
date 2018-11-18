package com.acmerobotics.roverruckus.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="auto")
public class opmode extends LinearOpMode {

    public void runOpMode() {
        waitForStart();
        while(opModeIsActive());
    }
}
