package com.acmerobotics.roverruckus.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.util.Range;

@Config
public class JoystickTransform {

    public static double dualThreshold = .5;
    public static double dualSlope = .5;
    public static double exponent = 2;
    public static double rampingTime = 1000;
    private boolean ramping = false;
    private Vector2d lastCommand = new Vector2d(0, 0);
    private long lastTime = 0;

    public MODE mode = MODE.LINEAR;
    public enum MODE {
        LINEAR,
        DUAL_ZONE,
        EXPONENTIAL
    }

    public JoystickTransform() {

    }

    public void setMode (MODE mode) {
        this.mode = mode;
    }

    public MODE getMode () {
        return mode;
    }

    public void setRamping (boolean enable) {
        ramping = enable;
    }

    public boolean getRamping () {
        return ramping;
    }

    public Vector2d transform(Vector2d original, Robot robot) {
        robot.addTelemetry("origX", original.getX());
        robot.addTelemetry("origY", original.getY());
        if (lastTime == 0) {
            lastTime = System.currentTimeMillis();
            return lastCommand;
        }
        long now = System.currentTimeMillis();
        long dt = now - lastTime;
        lastTime = now;

        double r = original.norm();
        robot.addTelemetry("r", r);

        switch (mode) {
            case LINEAR:
                break;
            case EXPONENTIAL:
                r = Math.pow(r, exponent);
                break;
            case DUAL_ZONE:
                if (r < dualThreshold)
                    r = dualSlope * r;
                else
                    r = -((1-dualSlope*dualThreshold) / (1 - dualThreshold)) * (r - 1) + 1;
        }
        Vector2d command = original.times(r / original.norm());
        if (r==0) command = new Vector2d(0, 0);
        if (!ramping) return command;

        Vector2d diff = command.minus(lastCommand);
        double dist = diff.norm();
        if (dist >= .01) {
            double max = dt / rampingTime;
            double delta = Range.clip(dist,0, max);
            command = command.plus(diff.times(delta / dist));
        }

        lastCommand = command;
        robot.addTelemetry("commandX", command.getX());
        robot.addTelemetry("commandY", command.getY());
        return command;
    }
}
