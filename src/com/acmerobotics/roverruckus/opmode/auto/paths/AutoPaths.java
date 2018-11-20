package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public abstract class AutoPaths {

    public Pose2d lander;

    public abstract Path landerToDepot();

    public abstract Path depotToPark();

    public static AutoPaths getPaths(Auto.START_LOCATION startLocaiton) {
        if (startLocaiton == Auto.START_LOCATION.CRATER) return new BasicCraterPaths();
        else return new BasicDepotPaths();
    }

}
