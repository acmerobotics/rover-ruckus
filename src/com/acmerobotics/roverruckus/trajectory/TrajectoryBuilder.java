package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roverruckus.util.SuperArrayList;

public class TrajectoryBuilder {

    private Waypoint lastWaypoint;
    private SuperArrayList<PathBuilder> paths;

    public TrajectoryBuilder (Waypoint start) {
        lastWaypoint = start;
        paths = new SuperArrayList<>();
        paths.add(new PathBuilder(lastWaypoint.getExit()));
    }

    public TrajectoryBuilder to (Waypoint waypoint) {

        paths.get(-1).splineTo(waypoint.getEnter(), new GoodLinearInterpolator(lastWaypoint.getHeading(), waypoint.getHeading()));
        if (waypoint.getStop()) paths.add(new PathBuilder(waypoint.getExit()));
        lastWaypoint = waypoint;
        return this;

    }

    public SuperArrayList<Trajectory> build () {
        SuperArrayList<Trajectory> trajectories = new SuperArrayList<>();
        for (PathBuilder path: paths) {
            trajectories.add(new Trajectory(path.build()));
        }
        return trajectories;
    }
}
