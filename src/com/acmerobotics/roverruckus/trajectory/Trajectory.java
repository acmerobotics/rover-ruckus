package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.opMode.auto.AutoFlag;

import java.util.ArrayList;

@Config
public abstract class Trajectory {

    public static double AXIAL_P = 6;
    public static double AXIAL_I = 0;
    public static double AXIAL_D = 0;
    public static double LATERAL_P = 6;
    public static double LATERAL_I = 0;
    public static double LATERAL_D = 0;
    public static double HEADING_P = 6;
    public static double HEADING_I = 0;
    public static double HEADING_D = 0;

    private ArrayList<AutoFlag> flags = new ArrayList<>();

    public void addFlag(AutoFlag flag) {
        flags.add(flag);
    }

    public boolean containsFlag(AutoFlag flag) {
        return flags.contains(flag);
    }

    public abstract Pose2d update(double t, Pose2d pose, TelemetryPacket packet);

    public abstract Pose2d getPose(double t);

    public abstract double getV(double t);

    public abstract double duration();

    public abstract boolean isComplete();

}
