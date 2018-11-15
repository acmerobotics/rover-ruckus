package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public abstract class Subsystem {

    abstract void update (TelemetryPacket packet) ;


}
