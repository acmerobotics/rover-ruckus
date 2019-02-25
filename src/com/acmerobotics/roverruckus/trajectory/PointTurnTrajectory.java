package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.acmerobotics.roverruckus.util.PIDController;

public class PointTurnTrajectory extends Trajectory {

    private Pose2d start;
    private MotionProfile profile;
    private double duration;
    private boolean complete;
    private PIDController headingController, trackingController;

    public PointTurnTrajectory(Pose2d start, double targetAngle) {
        this.start = start;
        System.out.println(start.getHeading());
        double turnAngle = Angle.norm(targetAngle - start.getHeading());
        System.out.println(turnAngle);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(turnAngle, 0, 0, 0),
                MecanumDrive.headingMaxV,
                MecanumDrive.headingMaxA,
                MecanumDrive.headingMaxJ
        );
        duration = profile.duration();
        complete = false;
        headingController = new PIDController(HEADING_P, HEADING_I, HEADING_D);
        trackingController = new PIDController(AXIAL_P, AXIAL_I, AXIAL_D);
    }

    @Override
    public Pose2d update(double t, Pose2d pose, TelemetryPacket packet) {
        if (t >= duration) complete = true;
        Pose2d targetState = getPose(t);
        Pose2d targetVelocity = new Pose2d(new Vector2d(), getV(t));

        double headingError = Angle.norm(pose.getHeading() - targetState.getHeading());
        Vector2d trackingError = pose.pos().minus(targetState.pos());
        double trackingErrorMag = trackingError.norm();

        double headingCorrection = headingController.update(headingError);
        double trackingCorrection = trackingController.update(trackingErrorMag);
        if (trackingErrorMag == 0) trackingErrorMag = 1;

        Pose2d correction = new Pose2d(trackingError.div(trackingErrorMag).times(trackingCorrection), headingCorrection);

        return targetVelocity.minus(correction);

    }

    @Override
    public Pose2d getPose(double t) {
        return new Pose2d(start.pos(), start.getHeading() + profile.get(t).getX());
    }

    @Override
    public double getV(double t) {
        return profile.get(t).getV();
    }

    @Override
    public double duration() {
        return duration;
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    public Path getPath() {
        return null;
    }
}
