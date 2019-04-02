package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.Differentiator;
import com.acmerobotics.roverruckus.util.LogWriter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class LiftTuner extends LinearOpMode {

    public static double a = .01;
    public static double maxHeight = 20;
    public static double minHeight = 5;

    @Override
    public void runOpMode () {
        Robot robot = new Robot (this, hardwareMap);
        LogWriter writer = new LogWriter("liftTuning", "position", "command", "velocity", "acceleration", "frame");
        Differentiator diff = new Differentiator();
        double lastCommand = 0;
        double direction = 1;

        waitForStart();

        while (opModeIsActive()) {
            if (robot.lift.getPosition() < minHeight) direction = 1;
            if (robot.lift.getPosition() > maxHeight) direction = -1;

            diff.update(robot.lift.getPosition());
            writer.writeLine((Double.toString(diff.getX()), Double.toString(lastCommand), Double.toString(diff.getV()), Double.toString(diff.getA()), Boolean.toString(robot.lift.frameIsAtBottom()));

            lastCommand += diff.getDt() * a * direction;
            robot.lift.setPower(lastCommand);

        }
    }
}
