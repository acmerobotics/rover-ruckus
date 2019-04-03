package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.Differentiator;
import com.acmerobotics.roverruckus.util.LogWriter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@Autonomous(name="liftTuner")
public class LiftTuner extends LinearOpMode {

    public static double a = .1;
    public static double maxHeight = 20;
    public static double minHeight = 5;

    @Override
    public void runOpMode () {
        Robot robot = new Robot (this, hardwareMap);
        LogWriter writer = new LogWriter("liftTuning", "position", "command", "velocity", "acceleration", "frame");
        Differentiator diff = new Differentiator();
        double lastCommand = 0;
        double direction = 1;
        double mid = (maxHeight + minHeight) / 2;
        boolean below = true;

        waitForStart();

        robot.lift.setPower(0);

        while (opModeIsActive()) {
            if (robot.lift.getPosition() < minHeight && !below) {
                below = true;
                direction = 1;
                lastCommand = 0;
                robot.lift.setPower(0);
                robot.pause(1000);
            }
            if (robot.lift.getPosition() > maxHeight && below) {
                below = false;
                direction = -1;
                lastCommand = 0;
                robot.lift.setPower(0);
                robot.pause(1000);
            }

            diff.update(robot.lift.getPosition());
            writer.writeLine(Double.toString(diff.getX()), Double.toString(lastCommand), Double.toString(diff.getV()), Double.toString(diff.getA()), Boolean.toString(robot.lift.frameIsAtBottom()));
            robot.addTelemetry("command", lastCommand);
            robot.addTelemetry("x", diff.getX());
            robot.addTelemetry("v", diff.getV());
            robot.addTelemetry("a", diff.getA());
            robot.addTelemetry("j", diff.getJ());

            lastCommand += diff.getDt() * a * direction;
            robot.lift.setPower(lastCommand);
            robot.update();

        }
    }
}
