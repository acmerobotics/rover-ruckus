package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.roverruckus.util.RoverRuckusConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="cyclingDepot")
public class CyclingAutoDepot extends AutoOpMode{
    @Override
    protected void run() {
        this.extendRake = () ->
                robot.intake.goToPosition(35);
        robot.intake.setRakeRetractBlockingDistance(1);
        startLocation = RoverRuckusConfiguration.StartLocation.DEPOT;
        robot.config.sampleBoth = false;
        AutoPaths paths = new AutoPaths(this, goldLocation, startLocation);
        robot.drive.setCurrentEstimatedPose(paths.start().pos());

        robot.drive.followTrajectories(paths.startToRelease());
        robot.waitForAllSubsystems();

        robot.drive.followTrajectory(paths.toSampleDepot().get(-1));
        robot.waitForAllSubsystems();
        deployMarker.execute();
        retractRake.execute();
        robot.waitForAllSubsystems();

        robot.drive.followTrajectories(paths.toPark());
        robot.waitForAllSubsystems();
    }

}
