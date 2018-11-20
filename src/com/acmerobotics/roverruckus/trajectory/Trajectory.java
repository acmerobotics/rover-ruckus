package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.acmerobotics.roverruckus.util.PIDController;

public class Trajectory {

    private Path path;
    private MotionProfile axialProfile;

    private PIDController axialController, lateralController, headingController;

    private double duration;
    private boolean complete = false;
    private double error = 0, axialError = 0, lateralError = 0, averageHeadingError = 0;


    public Trajectory (Path path) {

        this.path = path;
        this.axialProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(path.length(), 0, 0, 0),
                MecanumDrive.axialMaxV,
                MecanumDrive.axialMaxA,
                MecanumDrive.axialMaxJ
        );
        duration = axialProfile.duration();

        this.axialController = new PIDController(-1,0,0);
        this.lateralController = new PIDController(-1, 0, 0);
        this.headingController = new PIDController(-1, 0, 0);

    }

    long lastupdate = 0;

    public synchronized Pose2d update(double t, Pose2d pose, TelemetryPacket packet) {
        if (t >= duration) complete = true;
        Pose2d targetPose = path.get(axialProfile.get(t).getX());
        double theta = path.deriv(axialProfile.get(t).getX()).pos().angle();

        Pose2d targetVelocity = path.deriv(axialProfile.get(t).getX()).times(axialProfile.get(t).getV());

        Vector2d trackingError = pose.pos().minus(targetPose.pos()).rotated(-theta);
        packet.put("theta", theta);

        Vector2d trackingCorrection = new Vector2d(
                axialController.update(trackingError.getX()),
                lateralController.update(trackingError.getY())
        );

        double headingError = pose.getHeading() - targetPose.getHeading();
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

        return targetVelocity.plus(correction);
    }

    public synchronized boolean isComplete() {
        return complete;
    }

    public synchronized double averageError() {
        return complete ? error / duration: 0;
    }

    public synchronized double avergeLateralError() {return complete ? lateralError / duration: 0;}

    public synchronized double averageAxialError() {return complete ? axialError / duration: 0;}

    public synchronized double averageHeadingError() {return complete ? averageHeadingError / duration: 0;}

}
