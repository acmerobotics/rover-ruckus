package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.robot.Placer;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="rakeTest")
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
