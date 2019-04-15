package com.acmerobotics.roverruckus.opMode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;

@Autonomous(name="doubleSampleCrater")
public class DoubleSampleCraterAuto extends AutoOpMode {

    @Override
    protected void run() {
        this.extendRake = () -> robot.intake.goToPosition(30);
        robot.config.setSampleBoth(true);
        startLocation = StartLocation.CRATER;
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
