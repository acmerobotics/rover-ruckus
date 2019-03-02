package com.acmerobotics.roverruckus;

import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Clear Setpoints")
public class ClearSetpoints extends LinearOpMode {


    @Override public void runOpMode () {
        Robot robot = new Robot(this, hardwareMap);
        RobotState robotState = new RobotState(hardwareMap.appContext);

        waitForStart();

        robotState.setLiftOffset(robot.lift.getOffset());
        robotState.setLiftOffset(robot.lift.getOffset());
    }



}
