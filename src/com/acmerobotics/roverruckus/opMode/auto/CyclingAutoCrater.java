package com.acmerobotics.roverruckus.opMode.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;

@Autonomous(name="cyclingAutoCrater")
public class CyclingAutoCrater extends AutoOpMode{

    @Override
    protected void run() {
        this.extendRake = () ->
            robot.intake.goToPosition(40);
//        if (startLocation != StartLocation.CRATER) throw new IllegalArgumentException("incorrect config");
        startLocation = StartLocation.CRATER;
        robot.config.setSampleBoth(false);
        AutoPaths paths = new AutoPaths(this, goldLocation, startLocation);
        robot.drive.setCurrentEstimatedPose(paths.start().pos());

        robot.drive.followTrajectories(paths.startToRelease());
        robot.waitForAllSubsystems();

        robot.drive.followTrajectory(paths.toMarker().get(-1));
        robot.waitForAllSubsystems();
        deployMarker.execute();
        retractRake.execute();
        robot.waitForAllSubsystems();

        robot.drive.followTrajectories(paths.toSampleCrater());
        robot.pause(1000);
        robot.drive.followTrajectories(paths.toIntake());
        robot.waitForAllSubsystems();

        robot.drive.followTrajectories(paths.toScore());
        robot.waitForAllSubsystems();
        score.execute();
        robot.lift.liftBottom();
        robot.lift.setAsynch(false);
        robot.waitForAllSubsystems();

    }

}
