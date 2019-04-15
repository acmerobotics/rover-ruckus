package com.acmerobotics.roverruckus.trajectory;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.opMode.auto.AutoAction;
import com.acmerobotics.roverruckus.opMode.auto.AutoOpMode;

import java.util.ArrayList;
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

    public static double MAX_ACCEPTABLE_ERROR = 2;
    private Map<AutoAction, Double> actions;

    public Trajectory () {
        actions = new HashMap<>();
    }

    public synchronized Trajectory addAction (double t, AutoAction action) {
        if (t > duration()) t = duration();
        if (t < 0) t = duration() + t;
        actions.put(action, t);
        return this;


    }

    public Trajectory addAction (AutoAction action) {
        return addAction(0, action);
    }

    public Trajectory addActionOnCompletion (AutoAction action) {
        return addAction(duration(), action);
    }

    public synchronized Pose2d update(double t, Pose2d pose, TelemetryPacket packet) {
        for (AutoAction action: new ArrayList<>(actions.keySet())) {
            if (actions.get(action) <= t && getError() < MAX_ACCEPTABLE_ERROR) {
                action.execute();
                actions.remove(action);
                Log.i("trajectory", action.toString());
            }
        }
        return internalUpdate(t, pose, packet);
    }

    public abstract Pose2d internalUpdate (double t, Pose2d pose, TelemetryPacket packet);

    public abstract Pose2d getPose(double t);

    public abstract double getV(double t);

    public abstract double duration();

    public abstract boolean isComplete();

    public double getError () {
        return 0;
    }

    public abstract Path getPath();

}
