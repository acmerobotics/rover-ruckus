package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public abstract class AutoPaths {

    public static Pose2d lander;
    public static Pose2d sampling;
    public static Pose2d depot;
    public static Pose2d crater;

    public Path landerToSample() {
        return new PathBuilder(lander)
                .splineTo(sampling, new LinearInterpolator(lander.getHeading(), sampling.getHeading()))
                .build();
    }

    public Path sampleToDepot() {
        return new PathBuilder(sampling)
                .splineTo(depot, new LinearInterpolator(sampling.getHeading(), depot.getHeading()))
                .build();
    }

    public Path depotToPark() {
        return new PathBuilder(depot)
                .splineTo(crater, new LinearInterpolator(depot.getHeading(), crater.getHeading()))
                .build();
    }

    public static AutoPaths getPaths(Auto.START_LOCATION startLocaiton) {
        if (startLocaiton == Auto.START_LOCATION.CRATER) return new BasicCraterPaths();
        else return new BasicDepotPaths();
    }

}
