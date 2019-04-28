package com.acmerobotics.roverruckus.opMode;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Lift;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.robot.RobotState;
import com.acmerobotics.roverruckus.util.JoystickTransform;
import com.acmerobotics.roverruckus.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleop")
public class TeleOp extends LinearOpMode {

    private Robot robot;
    private JoystickTransform transform;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    @Override
    public void runOpMode() {
        Log.i("hash", Integer.toString(this.hashCode()));
        robot = new Robot(this, hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        transform = new JoystickTransform();
        robot.lift.setAsynch(false);

        boolean liftRaised = false;

        waitForStart();

        robot.lift.liftBottom();

        while (!isStopRequested()) {

            //drive
            Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
            robot.drive.setVelocity(v);
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robot.lift.setPosition(0);
                robot.intake.setPosition(0);
            }

            //lift
            //ratchet
            if (stickyGamepad2.left_bumper)
                robot.lift.engageRatchet();
            if (gamepad2.left_bumper)
                robot.lift.setVelocity(-1);
            else if (Math.abs(gamepad2.left_stick_y) > .1 || !robot.lift.isBusy()) {
                robot.lift.setVelocity(-gamepad2.left_stick_y);
            }
            if (gamepad2.right_bumper) robot.lift.disengageRatchet();

                //lift speed

            //lift positions
            if (gamepad2.dpad_up) {
                robot.lift.liftTop();
            }

            if (gamepad2.dpad_left) {
                robot.lift.findLatch();
            }

            if (gamepad2.dpad_down) {
                robot.lift.liftBottom();
            }

            if (gamepad2.dpad_right) robot.lift.dumpUp();

            //intake
            robot.intake.setIntakePower(gamepad2.left_trigger - gamepad2.right_trigger);

            if (stickyGamepad1.b) robot.intake.groundIntakererOut();
            else if (stickyGamepad1.a) robot.intake.groundIntakererIn();
            else if (stickyGamepad1.y) robot.intake.groundIntakererMiddle();

            //rake
            robot.intake.setArmPower(gamepad1.left_trigger - gamepad1.right_trigger);
            if (stickyGamepad1.right_bumper) robot.intake.toggleRake();
            if (stickyGamepad1.left_bumper) robot.intake.retractRake();

            //placer
            if (stickyGamepad2.a) robot.lift.placer.releaseSilver();
            if (stickyGamepad2.b) robot.lift.placer.releaseGold();
            if (gamepad2.x) robot.lift.placer.reset();

            robot.update();
            stickyGamepad1.update();
            stickyGamepad2.update();
        }
    }

}
