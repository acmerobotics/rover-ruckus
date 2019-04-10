package com.acmerobotics.roverruckus.util;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseUtil {

    public static double dot (Pose2d a, Pose2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY() + a.getHeading() + b.getHeading();
    }
}
