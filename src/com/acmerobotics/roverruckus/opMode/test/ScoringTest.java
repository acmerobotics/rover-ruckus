package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="scoringTest")
public class ScoringTest extends LinearOpMode {
    public static final String TAG = "scoringTest";

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        robot.lift.liftBottom();
        robot.waitForAllSubsystems();

        while (opModeIsActive()) {
            robot.intake.intake();
            robot.waitForAllSubsystems();
            robot.lift.liftTop();
            robot.waitForAllSubsystems();
            robot.lift.dumpUp();
            robot.pause(1000);
            robot.lift.placer.release();
            robot.waitForAllSubsystems();
            robot.pause(1000);
            robot.lift.liftBottom();
            robot.waitForAllSubsystems();
        }
    }
}
