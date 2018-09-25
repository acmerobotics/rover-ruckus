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

    private TelemetryPacket packet;

    public Trajectory (Path path) {

        this.path = path;
        this.axialProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(path.length(), 0, 0, 0),
                1,
                1,
                1
        );
        duration = axialProfile.duration();

        this.axialController = new PIDController();
        this.lateralController = new PIDController();
        this.headingController = new PIDController();

    }

    public Pose2d update(double t, Pose2d pose) {
        if (t >= duration) complete = true;
        Pose2d targetPose = path.get(axialProfile.get(t).getX());


        Pose2d targetVelocity = path.deriv(axialProfile.get(t).getX()).times(axialProfile.get(t).getV());

        Vector2d trackingError = pose.pos().minus(targetPose.pos()).rotated(targetPose.getHeading());

        Vector2d trackingCorrection = new Vector2d(
                axialController.update(trackingError.getX()),
                lateralController.update(trackingError.getY())
        );

        double headingError = pose.getHeading() - targetPose.getHeading();
        double headingCorrection = headingController.update(headingError);

        Pose2d correction = new Pose2d(trackingCorrection, headingCorrection);

        MecanumDrive.drawPose(packet.fieldOverlay(), pose, "black");
        MecanumDrive.drawPose(packet.fieldOverlay(), targetPose, "blue");

        packet.fieldOverlay().setStroke("red");

        packet.fieldOverlay().strokeLine(
                targetPose.getX(), targetPose.getY(),
                targetPose.getX() + trackingError.getX() * Math.cos(targetPose.getHeading()),
                targetPose.getY() + trackingError.getX() * Math.sin(targetPose.getHeading())
        );

        packet.fieldOverlay().strokeLine(
                targetPose.getX(), targetPose.getY(),
                targetPose.getX() + trackingError.getY() * Math.cos(targetPose.getHeading()),
                targetPose.getY() + trackingError.getY() * Math.sin(targetPose.getHeading())
        );

        packet.put("displacement", axialProfile.get(t).getX());
        packet.put("velocity", axialProfile.get(t).getV());
        packet.put("acceleration", axialProfile.get(t).getA());

        packet.put("axialError", trackingError.getX());
        packet.put("lateralError", trackingError.getY());
        packet.put("headingError", headingError);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        return targetVelocity.plus(correction);
    }

    public boolean isComplete() {
        return complete;
    }

}
