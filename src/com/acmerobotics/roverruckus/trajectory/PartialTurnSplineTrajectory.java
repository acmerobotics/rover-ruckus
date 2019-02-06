package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

public class PartialTurnSplineTrajectory extends SplineTrajectory {

    private PointTurnTrajectory turn;
    private Vector2d loc;

    public PartialTurnSplineTrajectory (Path path, double theta) {
        super (path);
        loc = path.start().pos();
        turn = new PointTurnTrajectory(path.start(), theta);
    }

    @Override
    public Pose2d update(double t, Pose2d pose, TelemetryPacket packet) {
        Vector2d pos = super.update(t, pose, packet).pos();
        double theta = turn.update(t, new Pose2d(loc, pose.getHeading()), packet).getHeading();
        return new Pose2d(pos, theta);

    }

    @Override
    public Pose2d getPose (double t) {
        return new Pose2d(super.getPose(t).pos(), turn.getPose(t).getHeading());
    }

    @Override
    public boolean isComplete() {
        return super.isComplete() && turn.isComplete();
    }

}
