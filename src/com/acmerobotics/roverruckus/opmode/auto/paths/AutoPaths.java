package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public abstract class AutoPaths {

    public abstract Path landerToSample ();

    public abstract Path sampleToDepot ();

    public abstract  Path depotToPark ();

    public static AutoPaths getPaths(Auto.START_LOCATION startLocaiton) {
        if (startLocaiton == Auto.START_LOCATION.CRATER) return new BasicCraterPaths();
        else return new BasicDepotPaths();
    }

}
