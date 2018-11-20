package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public class BasicCraterPaths extends AutoPaths{

//    static Pose2d sampling = new Pose2d(24+18,-18,  -Math.PI/4);
    static Pose2d sampling = new Pose2d(24, 0, 0);
    static Pose2d depot = new Pose2d(24, 24, 0);
    static Pose2d crater = new Pose2d(48, 24, Math.PI);
//    static Pose2d depot = new Pose2d(-60,36,  -3 * Math.PI / 4);
//    static Pose2d crater = new Pose2d(-60,-12,  0);

    public BasicCraterPaths() {
//        lander = new Pose2d(-12, -12, -3 * Math.PI/4);
        lander = new Pose2d(0, 0,0);

    }

    @Override
    public Path landerToDepot() {
        return new PathBuilder(lander)
                .splineTo(sampling, new LinearInterpolator(lander.getHeading(), sampling.getHeading()))
                .splineTo(depot, new LinearInterpolator(sampling.getHeading(), depot.getHeading()))
                .build();
    }

    @Override
    public Path depotToPark() {
        return new PathBuilder(depot)
                .splineTo(crater, new LinearInterpolator(depot.getHeading(), crater.getHeading()))
                .build();
    }
}