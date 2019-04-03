package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Lift;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="demo")
public class Demo extends LinearOpMode {

    @Override
    public void runOpMode () {

        telemetry.addLine("initializing");
        telemetry.update();

        Robot robot = new Robot (this, hardwareMap);
        MecanumDrive.TELEOP_V = 15;
        robot.intake.retractRake();
        robot.lift.liftBottom();
        robot.waitForAllSubsystems();
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.clear();
        telemetry.addLine("Initialized. Press Play to start");
        telemetry.update();

        waitForStart();

        telemetry.clear();
        telemetry.update();

        while (!isStopRequested()) {
            if (gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0) {
                robot.drive.setVelocity(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
            } else {
                robot.drive.setVelocity(new Pose2d(-gamepad2.left_stick_x, -gamepad2.left_stick_y, -gamepad2.right_stick_x));
            }

            robot.intake.setIntakePower(gamepad1.left_trigger - gamepad1.right_trigger);

            if (stickyGamepad1.dpad_up) {
                robot.lift.liftTop();
                robot.lift.placer.closeIntake();
            }

            if (stickyGamepad1.dpad_left) robot.lift.goToPosition(Lift.LIFT_LATCH);

            if (stickyGamepad1.dpad_down) {
                robot.lift.liftBottom();
                robot.lift.placer.reset();
            }

            if (stickyGamepad1.dpad_right) robot.lift.dumpUp();

            if (gamepad1.left_bumper && !gamepad2.left_bumper) {
                robot.lift.setVelocity(-1);
            }

            if (gamepad2.a) robot.lift.placer.releaseSilver();
            if (gamepad2.b) robot.lift.placer.releaseGold();
            if (gamepad2.x) robot.lift.placer.reset();

            if (stickyGamepad1.left_bumper) robot.lift.engageRatchet();

            if (stickyGamepad2.right_bumper) robot.lift.disengageRatchet();

            stickyGamepad1.update();
            stickyGamepad1.update();
        }

    }

}
