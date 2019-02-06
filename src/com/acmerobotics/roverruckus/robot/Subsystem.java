package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public abstract class Subsystem {

    abstract void update(TelemetryPacket packet);

    public boolean isBusy() {
        return false;
    }

    public void waitForCompleteion() {
        for (; ; ) {
            if (!isBusy()) return;
            try {
                Thread.sleep(50);
            } catch (InterruptedException ie) {
                //yikes
            }
        }
    }
}
