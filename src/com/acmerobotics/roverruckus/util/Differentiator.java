package com.acmerobotics.roverruckus.util;

import com.acmerobotics.roadrunner.profile.MotionState;

public class Differentiator {

    private MotionState lastState;
    private long lastTime;
    private boolean initialized = false;
    private double dt = 0;

    public MotionState update (double position) {
        if (!initialized) {
            lastState = new MotionState(position, 0, 0, 0);
            lastTime = System.nanoTime();
            initialized = true;
        }

        long now = System.nanoTime();
        dt = (now - lastTime) / 1.0e9;
        lastTime = now;

        double velocity = (position - lastState.getX()) / dt;
        double acceleration = (velocity - lastState.getV()) / dt;
        double jerk = (acceleration - lastState.getA()) / dt;

        lastState = new MotionState(position, velocity, acceleration, jerk);
        return lastState;
    }

    public double getDt () {
        return dt;
    }

    public double getX () {
        return lastState.getX();
    }

    public double getV () {
        return lastState.getV();
    }

    public double getA () {
        return lastState.getA();
    }

    public double getJ () {
        return lastState.getJ();
    }

    public MotionState getState () {
        return lastState;
    }
}
