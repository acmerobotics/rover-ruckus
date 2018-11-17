package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public class BasicDepotPaths extends AutoPaths{


    @Override
    public Path landerToSample(Auto.START_LOCATION start) {
        return null;
    }

    @Override
    public Path sampleToDepot(Auto.START_LOCATION start) {
        return null;
    }

    @Override
    public Path depotToPark(Auto.START_LOCATION start) {
        return null;
    }
}
