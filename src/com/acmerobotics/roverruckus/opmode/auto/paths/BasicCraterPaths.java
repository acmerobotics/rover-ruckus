package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public class BasicCraterPaths extends AutoPaths{

    public BasicCraterPaths() {
        lander = new Pose2d(12, 12, -Math.PI/4);
        sampling = new Pose2d(36, 12, Math.PI/4);
        depot = new Pose2d(5*12, 3*12, -Math.PI*(3.0/4.0));
    }
}