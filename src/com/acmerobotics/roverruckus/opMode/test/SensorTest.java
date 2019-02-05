package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="hall effect test")
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode () {
        Robot robot = new Robot (this, hardwareMap);
        while (!isStopRequested()) {
            robot.addTelemetry("yup", robot.lift.isSensor());
            robot.update();
        }
    }
}
