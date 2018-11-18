package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="RakeTest")
public class RakeTest extends LinearOpMode{

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            robot.intake.setArmPower(gamepad1.left_stick_y);

            if (gamepad1.a) robot.intake.doSort();
            if (gamepad1.b) robot.intake.resetSorter();

            if (gamepad1.left_bumper) robot.intake.intakeForward();
            else if (gamepad1.left_trigger > 0) robot.intake.intakeBackward();
            else if (gamepad1.right_bumper) robot.intake.intakeStop();
        }
    }

}
