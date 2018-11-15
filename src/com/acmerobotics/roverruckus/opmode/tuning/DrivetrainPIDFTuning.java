package com.acmerobotics.roverruckus.opmode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.util.CautiousBinarySearch;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.code.ConservativeTranslationAdvice;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

@Config
@Autonomous(name="drivetrainPIDFTuning")
public class DrivetrainPIDFTuning extends LinearOpMode{

    public static double p, i, d, f;

    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);

        PIDFCoefficients pidfCoefficients = robot.drive.getMotorPIDF();
        p = pidfCoefficients.p;
        i = pidfCoefficients.i;
        d = pidfCoefficients.d;
        f = pidfCoefficients.f;

        robot.addTelemetry("p", p);
        robot.addTelemetry("i", i);
        robot.addTelemetry("d", d);
        robot.addTelemetry("f", f);

        CautiousBinarySearch search = new CautiousBinarySearch(10, 20, .1);

        waitForStart();

        f = 15.8;

        List<Vector2d> deltas = new ArrayList<>();

        deltas.add(new Vector2d(0, 48));
        deltas.add(new Vector2d(0, -48));

        while (!isStopRequested()) {




            double averageError = 0;
            for (Vector2d delta: deltas) {
                robot.drive.followPath(new PathBuilder(robot.drive.getCurrentEstimatedPose())
                        .lineTo(robot.drive.getCurrentEstimatedPose().pos().plus(delta))
                        .build());

                while (robot.drive.isFollowingPath()) {
                    if (isStopRequested()) {
                        robot.drive.stop();
                        return;
                    }

                }
                double error = robot.drive.trajectory.averageAxialError();
                averageError += error * (1.0 / deltas.size());
            }

            //f = search.update(averageError > 0);
            robot.addTelemetry("averageError", averageError);
            robot.addTelemetry("lower", search.getLower());
            robot.addTelemetry("upper", search.getUpper());
            robot.addTelemetry("f", f);

            robot.drive.setMotorPIDF(p, i, d ,f);

        }

        robot.stop();
    }

}
