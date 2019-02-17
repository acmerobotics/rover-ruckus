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

public class SplineTrajectory extends Trajectory {

    private Path path;
    private MotionProfile axialProfile;

    private PIDController axialController, lateralController, headingController;

    private double duration;
    private boolean complete = false;
    private double error = 0, axialError = 0, lateralError = 0, averageHeadingError = 0;

    public SplineTrajectory(Path path) {
        this(path, 0, false);
    }

    public SplineTrajectory(Path path, double startAccelerate, boolean stopAccelerate) {

        this.path = path;
        this.axialProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, startAccelerate, 0),
                new MotionState(path.length(), 0, stopAccelerate ? -MecanumDrive.axialMaxA : 0, 0),
                MecanumDrive.axialMaxV,
                MecanumDrive.axialMaxA,
                MecanumDrive.axialMaxJ
        );
        duration = axialProfile.duration();

        this.axialController = new PIDController(AXIAL_P, AXIAL_I, AXIAL_D);
        this.lateralController = new PIDController(LATERAL_P, LATERAL_I, LATERAL_D);
        this.headingController = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    }

    long lastupdate = 0;

    @Override
    public synchronized Pose2d update(double t, Pose2d pose, TelemetryPacket packet) {
        if (t >= duration) complete = true;
        Pose2d targetPose = path.get(axialProfile.get(t).getX());
        double theta = Angle.norm(path.deriv(axialProfile.get(t).getX()).pos().angle());

        Pose2d targetVelocity = path.deriv(axialProfile.get(t).getX()).times(axialProfile.get(t).getV());
        Vector2d trackingError = pose.pos().minus(targetPose.pos()).rotated(-theta);

        packet.fieldOverlay().setStroke("green");
        packet.fieldOverlay().strokeLine(pose.getX(), pose.getY(), pose.getX() + targetVelocity.getX(), pose.getY() + targetVelocity.getY());

        packet.put("theta", theta);

        Vector2d trackingCorrection = new Vector2d(
                axialController.update(trackingError.getX()),
                lateralController.update(trackingError.getY())
        );
        trackingCorrection = trackingCorrection.rotated(theta);
        packet.fieldOverlay().setStroke("pink");
        packet.fieldOverlay().strokeLine(pose.getX(), pose.getY(), pose.getX() + trackingCorrection.getX(), pose.getY() + trackingCorrection.getY());

        double headingError = Angle.norm(pose.getHeading() - targetPose.getHeading());
        double headingCorrection = headingController.update(headingError);

        Pose2d correction = new Pose2d(trackingCorrection, headingCorrection);

        MecanumDrive.drawPose(packet.fieldOverlay(), targetPose, "blue");

        packet.fieldOverlay().setStroke("red");

        packet.fieldOverlay().strokeLine(
                targetPose.getX(), targetPose.getY(),
                targetPose.getX() + trackingError.getX() * Math.cos(theta),
                targetPose.getY() + trackingError.getX() * Math.sin(theta)
        );

        packet.fieldOverlay().setStroke("purple");

        packet.fieldOverlay().strokeLine(
                targetPose.getX(), targetPose.getY(),
                targetPose.getX() + trackingError.getY() * -Math.sin(theta),
                targetPose.getY() + trackingError.getY() * Math.cos(theta)
        );

        packet.put("displacement", axialProfile.get(t).getX());
        packet.put("velocity", axialProfile.get(t).getV());
        packet.put("acceleration", axialProfile.get(t).getA());


        packet.put("axialError", trackingError.getX());
        packet.put("lateralError", trackingError.getY());
        packet.put("headingError", headingError);

        if (lastupdate == 0) lastupdate = System.currentTimeMillis();
        else {
            long now = System.currentTimeMillis();
            long dt = now - lastupdate;
            lastupdate = now;
            error += trackingError.norm() * dt;
            axialError += trackingError.getX() * dt;
            lateralError += trackingError.getY() * dt;
            averageHeadingError += headingError * dt;
        }

        return targetVelocity.minus(correction);
    }

    @Override
    public synchronized boolean isComplete() {
        return complete;
    }

//    public synchronized double averageError() {
//        return complete ? error / duration: 0;
//    }
//
//    public synchronized double avergeLateralError() {return complete ? lateralError / duration: 0;}
//
//    public synchronized double averageAxialError() {return complete ? axialError / duration: 0;}
//
//    public synchronized double averageHeadingError() {return complete ? averageHeadingError / duration: 0;}

    @Override
    public Pose2d getPose(double t) {
        return path.get(axialProfile.get(t).getX());
    }

    @Override
    public double getV(double t) {
        return axialProfile.get(t).getV();
    }

    @Override
    public double duration() {
        return axialProfile.duration();
    }

    @Override
    public com.acmerobotics.roadrunner.path.Path getPath() {
        return path;
    }
}
