package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roverruckus.opMode.auto.AutoFlag;
import com.acmerobotics.roverruckus.util.SuperArrayList;

public class TrajectoryBuilder {

    private Waypoint lastWaypoint;
    private SuperArrayList<Trajectory> trajectories;
    private PathBuilder currentPath;
    private boolean added = false;

    public TrajectoryBuilder (Waypoint start) {
        lastWaypoint = start;
        trajectories = new SuperArrayList<>();
        currentPath = new PathBuilder(lastWaypoint.getExit());
    }

    public TrajectoryBuilder to (Waypoint waypoint) {

        currentPath.splineTo(waypoint.getEnter(), new GoodLinearInterpolator(lastWaypoint.getHeading(), waypoint.getHeading()));
        added = true;
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

    public TrajectoryBuilder partialTurnTo (Waypoint waypoint) {
        newPath();
        PathBuilder path = new PathBuilder(lastWaypoint.getExit());
        path.splineTo(waypoint.getEnter(), new ConstantInterpolator(lastWaypoint.getHeading()));
        trajectories.add(new PartialTurnSplineTrajectory(path.build(), waypoint.getHeading()));
        return this;
    }

    private void newPath () {
        if (added) trajectories.add(new SplineTrajectory(currentPath.build()));
        added = false;
        currentPath = new PathBuilder(lastWaypoint.getExit());
    }

    public TrajectoryBuilder addFlag (AutoFlag flag) {
        trajectories.get(-1).addFlag(flag);
        return this;

    }

    public SuperArrayList<Trajectory> build () {
        newPath();
        return trajectories;
    }
}
