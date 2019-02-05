package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roverruckus.util.SuperArrayList;

public class TrajectoryBuilder {

    private Waypoint lastWaypoint;
    private SuperArrayList<Trajectory> trajectories;
    private PathBuilder currentPath;

    public TrajectoryBuilder (Waypoint start) {
        lastWaypoint = start;
        trajectories = new SuperArrayList<>();
        currentPath = new PathBuilder(lastWaypoint.getExit());
    }

    public TrajectoryBuilder to (Waypoint waypoint) {

        currentPath.splineTo(waypoint.getEnter(), new GoodLinearInterpolator(lastWaypoint.getHeading(), waypoint.getHeading()));
        lastWaypoint = waypoint;
        if (waypoint.getStop()) newPath();
        return this;

    }

    public TrajectoryBuilder turnTo (double heading) {
        PointTurnTrajectory trajectory = new PointTurnTrajectory(lastWaypoint.pos(), heading);
        lastWaypoint = new Waypoint(new Pose2d(lastWaypoint.pos().pos(), heading), lastWaypoint.getExit().getHeading());
        newPath();
        trajectories.add(trajectory);
        return this;

    }

    private void newPath () {
        trajectories.add(new SplineTrajectory(currentPath.build()));
        currentPath = new PathBuilder(lastWaypoint.getExit());
    }

    public SuperArrayList<Trajectory> build () {
        return trajectories;
    }
}
