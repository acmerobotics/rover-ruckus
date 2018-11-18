package com.acmerobotics.roverruckus.opmode.auto.paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.opmode.auto.Auto;

public class BasicDepotPaths extends AutoPaths{

    public BasicDepotPaths() {
        lander = new Pose2d(12, -12, Math.PI / 4);
        sampling = new Pose2d(24, 48, Math.PI / 2);
        depot = new Pose2d(12 * 3, 12 * 5, Math.PI * (3.0 / 4.0));
        crater = new Pose2d(-12, 12 * 5, Math.PI);
    }

}
