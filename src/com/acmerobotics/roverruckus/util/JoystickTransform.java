package com.acmerobotics.roverruckus.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class JoystickTransform {

    public static double dualThreshold = .5;
    public static double dualSlope = .5;
    public static double exponent = 2;
    public static double rampingTime = 1000;
    private boolean ramping = false;
    private Pose2d lastCommand = new Pose2d(0, 0, 0);
    private long lastTime = 0;

    public MODE mode = MODE.LINEAR;

    public enum MODE {
        LINEAR,
        DUAL_ZONE,
        EXPONENTIAL
    }

    public JoystickTransform() {

    }

    public void setMode(MODE mode) {
        this.mode = mode;
    }

    public MODE getMode() {
        return mode;
    }

    public void setRamping(boolean enable) {
        ramping = enable;
    }

    public boolean getRamping() {
        return ramping;
    }

    public Pose2d transform(Pose2d original) {
        if (lastTime == 0) {
            lastTime = System.currentTimeMillis();
            return lastCommand;
        }
        long now = System.currentTimeMillis();
        long dt = now - lastTime;
        lastTime = now;

        double r = original.pos().norm();
        double omega = original.getHeading();
        double sigOmega = Math.signum(omega);
        omega = Math.abs(omega);

        switch (mode) {
            case LINEAR:
                break;
            case EXPONENTIAL:
                r = Math.pow(r, exponent);
                omega = Math.pow(omega, exponent);
                break;
            case DUAL_ZONE:
                if (r < dualThreshold)
                    r = dualSlope * r;
                else
                    r = ((1 - dualSlope * dualThreshold) / (1 - dualThreshold)) * (r - 1) + 1;
                if (omega < dualThreshold)
                    omega = dualSlope * omega;
                else
                    omega = ((1 - dualSlope * dualThreshold) / (1 - dualThreshold)) * (omega - 1) + 1;

        }
        Pose2d command = new Pose2d(original.pos().times(r / original.pos().norm()), omega * sigOmega);
        if (r == 0) command = new Pose2d(0, 0, omega * sigOmega);
        return command;
    }
}
