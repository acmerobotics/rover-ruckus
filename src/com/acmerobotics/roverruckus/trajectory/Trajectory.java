package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.opMode.auto.AutoAction;
import com.acmerobotics.roverruckus.opMode.auto.AutoOpMode;

import java.util.HashMap;
import java.util.Map;

@Config
public abstract class Trajectory {

    public static double AXIAL_P = 2;
    public static double AXIAL_I = 0;
    public static double AXIAL_D = 0;
    public static double LATERAL_P = 2;
    public static double LATERAL_I = 0;
    public static double LATERAL_D = 0;
    public static double HEADING_P = 6;
    public static double HEADING_I = 0;
    public static double HEADING_D = 0;
    public static double K_A = 0;

    private Map<AutoAction, Double> actions;

    public Trajectory () {
        actions = new HashMap<>();
    }

    public Trajectory addAction (double t, AutoAction action) {
        if (t < 0 || t > duration()) t = duration();
        actions.put(action, t);
        return this;
    }

    public Trajectory addAction (AutoAction action) {
        return addAction(0, action);
    }

    public Trajectory addActionOnCompletion (AutoAction action) {
        return addAction(-1, action);
    }

    public Pose2d update(double t, Pose2d pose, TelemetryPacket packet) {
        for (AutoAction action: actions.keySet())
            if (actions.get(action) >= t) action.execute();
        return internalUpdate(t, pose, packet);
    }

    public abstract Pose2d internalUpdate (double t, Pose2d pose, TelemetryPacket packet);

    public abstract Pose2d getPose(double t);

    public abstract double getV(double t);

    public abstract double duration();

    public abstract boolean isComplete();

    public abstract Path getPath();

}
