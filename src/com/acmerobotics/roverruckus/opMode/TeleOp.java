package com.acmerobotics.roverruckus.opMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.robot.Lift;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.JoystickTransform;
import com.acmerobotics.roverruckus.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop")
public class TeleOp extends LinearOpMode {

    private Robot robot;
    private JoystickTransform transform;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    @Override
    public void runOpMode() {
       robot = new Robot(this, hardwareMap);
       stickyGamepad1 = new StickyGamepad(gamepad1);
       stickyGamepad2 = new StickyGamepad(gamepad2);
       transform = new JoystickTransform();
       robot.lift.placer.setEnabled(false);
//       transform.setMode(JoystickTransform.MODE.DUAL_ZONE);

       waitForStart();

       while (!isStopRequested()) {

           //drive
           Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
           robot.drive.setVelocity(v);
           robot.addTelemetry("x", v.getX());
           robot.addTelemetry("y", v.getY());
           robot.addTelemetry("z", v.getHeading());
//           if (stickyGamepad1.a) robot.drive.holdPosition();
           if (stickyGamepad2.left_bumper)
               robot.lift.engageRatchet();

           if (gamepad2.left_bumper) {
               robot.lift.setVelocity(-1);
           }
           else if (Math.abs(gamepad2.left_stick_y) > .1 || !robot.lift.isBusy())
               robot.lift.setVelocity(-gamepad2.left_stick_y);

           if (gamepad2.right_bumper) robot.lift.disengageRatchet();
           if (gamepad2.dpad_up) {
               robot.lift.placer.setEnabled(false);
               robot.lift.liftTop();
               robot.lift.placer.closeIntake();
           }
           if (gamepad2.dpad_left) robot.lift.goToPosition(Lift.LIFT_LATCH);
           if (gamepad2.dpad_down) {
               robot.lift.liftBottom();
               robot.lift.placer.reset();
           }
           if (gamepad2.dpad_right) robot.lift.dumpUp();
           //rake
           double rakeVelocity = gamepad2.right_stick_y;
           robot.intake.setArmPower(rakeVelocity);
           if (gamepad2.y) robot.intake.rakeUp();
//
//           //intake
           robot.intake.setIntakePower(gamepad2.left_trigger - gamepad2.right_trigger);
           if (gamepad2.left_trigger > .1) robot.lift.placer.setEnabled(true);
           robot.intake.setArmPower(gamepad1.left_trigger - gamepad1.right_trigger);
           if (stickyGamepad1.right_bumper) robot.intake.toggleRake();
           if (stickyGamepad1.left_bumper) robot.intake.retractRake();

           //lift
//           if (gamepad2.a) robot.lift.placer.releaseFirst();
//           if (gamepad2.b) robot.lift.placer.releaseSecond();
           if (stickyGamepad2.a) robot.lift.placer.releaseSilver();
           if (stickyGamepad2.b) robot.lift.placer.releaseGold();
           if (gamepad2.x) robot.lift.placer.reset();
           if (gamepad2.y) robot.lift.goToPosition(Lift.LIFT_LATCH);

           robot.update();
           stickyGamepad1.update();
           stickyGamepad2.update();
       }
    }

}
