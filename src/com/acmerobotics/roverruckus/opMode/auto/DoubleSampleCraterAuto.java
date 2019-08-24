package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.roverruckus.util.RoverRuckusConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="doubleSampleCrater")
public class DoubleSampleCraterAuto extends AutoOpMode {

    @Override
    protected void run() {
        this.extendRake = () -> robot.intake.goToPosition(30);
        robot.config.sampleBoth = true;
        startLocation = RoverRuckusConfiguration.StartLocation.CRATER;
        AutoPaths paths = new AutoPaths(this, goldLocation, startLocation);
        robot.drive.setCurrentEstimatedPose(paths.start().pos());

        robot.drive.followTrajectories(paths.startToRelease());
        robot.intake.setPosition(0);

        robot.drive.followTrajectories(paths.toSampleBoth());
        robot.intake.deployMarker();
        robot.intake.retractRake();
        robot.waitForAllSubsystems();

        robot.drive.followTrajectories(paths.toSampleCrater());
        robot.waitForAllSubsystems();

        robot.pause(1000);
        robot.drive.followTrajectories(paths.toIntake());
        robot.waitForAllSubsystems();

    }
}
