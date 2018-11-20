package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public class BasicDepotPaths extends AutoPaths{

    static Pose2d sampling = new Pose2d(-24,48,  Math.PI / 4);
    static Pose2d depot = new Pose2d(-36,60,  Math.PI/4);
    static Pose2d crater = new Pose2d(24,60,  -Math.PI/2);

    public BasicDepotPaths() {
        lander = new Pose2d(-12, 12,  -Math.PI / 4);

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
