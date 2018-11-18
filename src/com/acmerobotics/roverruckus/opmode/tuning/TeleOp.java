package com.acmerobotics.roverruckus.opmode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop")
public class TeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap);
    }

    @Override
    public void loop() {
        //drivetrain
        robot.drive.setVelocity(new Pose2d(
                -gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x
        ));

        //lift
        if (gamepad1.a) robot.lift.latch();
        else if (gamepad1.b) robot.lift.unlatch();
        robot.lift.setVelocty(gamepad1.right_trigger - gamepad1.left_trigger);

        //intake
        robot.intake.setIntakePower(gamepad2.left_trigger - gamepad2.right_trigger);
        if (gamepad2.dpad_down) robot.intake.rakeStow();
        else if (gamepad2.dpad_left) robot.intake.rakeDown();
        else if (gamepad2.dpad_up) robot.intake.rakeUp();
        else robot.intake.setRakeVelocity(gamepad2.left_stick_x);
        if (gamepad2.a) robot.intake.dumpDump();
        else robot.intake.resetDump();
        robot.intake.setArmPower(gamepad2.right_stick_y);

        robot.updateSubsystems();
        robot.updateTelemetry();
    }

}
