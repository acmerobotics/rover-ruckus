package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.roadrunner.Pose2d;

public class Waypoint {

    private Pose2d pose;
    private double enter, exit;
    private boolean stop;

    public Waypoint(Pose2d pose, double enter, double exit, boolean stop) {
        this.pose = pose;
        this.enter = enter;
        this.exit = exit;
        this.stop = enter != exit || stop;
    }

    public Waypoint(Pose2d pose, double angle) {
        this(pose, angle, angle, false);
    }

    public Waypoint(Pose2d pose, double angle, boolean stop) {
        this(pose, angle, angle, stop);
    }

    public Waypoint(Pose2d pose, double enter, double exit) {
        this(pose, enter, exit, enter != exit);
    }

    public Pose2d getEnter() {
        return new Pose2d(pose.pos(), enter);
    }

    public Pose2d getExit() {
        return new Pose2d(pose.pos(), exit);
    }

    public double getHeading() {
        return pose.getHeading();
    }

    public Pose2d pos() {
        return pose;
    }

    public boolean getStop() {
        return stop;
    }
}
